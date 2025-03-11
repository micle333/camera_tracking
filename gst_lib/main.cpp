#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <getopt.h>
#include <stdint.h>
#include <unistd.h>
#include <sys/time.h>
#include <math.h>
#include <time.h>
#include <iostream>
#include <chrono>
#include <string>
#include <sstream>
#include <opencv2/opencv.hpp>

#include <gst/gst.h>

#define ARRAY_LEN(_x) ((int)(sizeof(_x) / sizeof(_x[0])))

#define MAX_DEV 13

//! The minimum frames between init and watchdog to consider init not poor (less -> poor init)
#define POOR_INIT_FRAMES_LIMIT 25u

//! The number of consequitive poor inits after which the stream is locked permanently
#define MAX_POOR_INITS_BEFORE_LOCK 5u

GstClockTime base_time;

static uint8_t verbose = 1;

struct camera_info
{
    /* device info */
    std::string device{"/dev/video0"};
    int16_t width{1920};
    int16_t height{1080};
    char pixel_format[10]{"UYVY"};
    int16_t rotation{};
    int16_t frame_rate{30};
    uint64_t poor_inits_count{3};
    uint64_t frames_count{};
    uint64_t frames_after_init_count{1};
    uint64_t prev_frames_count{};
    uint64_t prev_time_sec{};
    double stat_mb{};
    /* watchdog statistics */
    uint64_t last_frame_ts_ms{};
    uint16_t restarts{};
    uint64_t init_time_ms{};
    uint64_t max_duration_ms{500};
    char max_duration_str[256];
    uint64_t startNs{0};
};

static volatile int stop = 0;
void sig_handler(int sig)
{
    (void)sig;
    stop = 1;
}

uint64_t nowNs()
{
    const auto duration{std::chrono::system_clock::now().time_since_epoch()};
    return std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count();
}

static void msec_to_str(char *str, uint64_t ms)
{

    int seconds = ms / 1000;
    ms %= 1000;
    int minutes = seconds / 60;
    seconds %= 60;
    int hours = minutes / 60;
    minutes %= 60;
    int days = hours / 24;
    hours %= 24;
    sprintf(str, "%02dd:%02dh:%02dm:%02ds", days, hours, minutes, seconds);
}

static GstFlowReturn new_sample(GstElement *sink, struct camera_info *cam_info)
{

    GstSample *sample;

    /* Retrieve the buffer */
    g_signal_emit_by_name(sink, "pull-sample", &sample);

    if (sample)
    {
        cam_info->frames_count++;
        cam_info->frames_after_init_count++;

        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC_RAW, &now);

        uint64_t time_sec = now.tv_sec;
        // uint64_t time_usec = now.tv_nsec / 1000;
        cam_info->last_frame_ts_ms = now.tv_sec * 1000 + lround(now.tv_nsec / 1e6);

        GstBuffer *buf = gst_sample_get_buffer(sample);

        if (buf)
        {
            GstMapInfo info;
            gst_buffer_map(buf, &info, GST_MAP_READ);

            GstCaps *caps = gst_sample_get_caps(sample);

            cam_info->stat_mb += (double)info.size / 1024 / 1024;

            // if (verbose && (time_sec - cam_info->prev_time_sec >= 1))
            {
                cam_info->prev_time_sec = time_sec;
                cam_info->prev_frames_count = cam_info->frames_count;
                // fflush(stdout);

                GstStructure *s = gst_caps_get_structure(caps, 0);
                int imW, imH;
                if (caps != nullptr)
                {
                    gst_structure_get_int(s, "width", &imW);
                    gst_structure_get_int(s, "height", &imH);

                    // const auto buffer_ts = GST_BUFFER_PTS(buf);
                    // const auto monotonic_ts = buffer_ts + base_time;

                    // printf("Buffer TS: %" GST_TIME_FORMAT
                    //        " is monotonic TS: %" GST_TIME_FORMAT "\n",
                    //        GST_TIME_ARGS(buffer_ts),
                    //        GST_TIME_ARGS(monotonic_ts));

                    // std::cout << "size = " << info.size << " ==? " << imW * imH * 3 << std::endl;
                    if (cam_info->startNs == 0)
                        cam_info->startNs = nowNs();

                    const auto buffer_ts{GST_BUFFER_PTS(buf)};
                    const time_t t{int64_t((buffer_ts + cam_info->startNs) / 1e9)};
                    std::cout << "Frame: " << ctime(&t) << " " << imW << " " << imH << " " << info.size << std::endl;

                    // cv::Mat mat_src = cv::Mat(imH, imW, CV_8UC2, info.data);
                    // cv::Mat mat_dst = cv::Mat(imH, imW, CV_8UC3);

                    // cv::Mat mat_dst = cv::Mat(imH, imW, CV_8UC4, info.data);
                    cv::Mat mat_dst = cv::Mat(imH, imW, CV_8UC3, info.data);

                    // cv::cvtColor(mat_src, mat_dst, cv::COLOR_YUV2RGBA_UYVY);

                    // std::cout << "Frame: " << mat_dst.size() << std::endl;
                    // if (not mat_dst.empty())
                    //     cv::imwrite("./img.jpg", mat_dst);
                    if (!mat_dst.empty()) {
                        cv::imshow("Camera", mat_dst);
                        cv::waitKey(1);  // Минимальная задержка для отображения
                    }
                }
            }

            gst_buffer_unmap(buf, &info);
        }
        else
        {
            g_print("no buffer in sample\n");
        }

        gst_sample_unref(sample);
        return GST_FLOW_OK;
    }

    return GST_FLOW_ERROR;
}

static void *run_pipeline(camera_info &info)
{
    GstElement *pipeline, *sink;
    GstBus *bus;
    GstMessage *msg;
    GstStateChangeReturn ret;

    struct timespec now;

    info.poor_inits_count = 0;

init:
    clock_gettime(CLOCK_MONOTONIC_RAW, &now);

    info.last_frame_ts_ms = 0;

    info.frames_after_init_count = 0;

    info.init_time_ms = now.tv_sec * 1000 + lround(now.tv_nsec / 1e6);

    GError *error = NULL;
    const std::string launchStr{
        "nvv4l2camerasrc do-timestamp=1 name=src device=" + info.device + " ! video/x-raw(memory:NVMM), width=" +
        std::to_string(info.width) + ", height=" + std::to_string(info.height) +
        ", framerate=" + std::to_string(info.frame_rate) + "/1 ! nvvidconv ! video/x-raw,format=BGRx ! videoconvert ! video/x-raw,format=BGR" +
        " ! appsink name=sink max-buffers=2 sync=0"};

    // caps=video/x-raw,format=BGRx"};

    //! nvvidconv ! video/x-raw,format=BGRx ! videoconvert ! video/x-raw,format=BGR" +
    //" ! appsink name=sink max-buffers=2 sync=0"};

    pipeline = gst_parse_launch(launchStr.c_str(), &error);
    if (error != NULL)
    {
        g_print("Could not construct pipeline: %s\n", error->message);
        g_error_free(error);
        return {};
    }

    sink = gst_bin_get_by_name(GST_BIN(pipeline), "sink");
    g_object_set(G_OBJECT(sink), "emit-signals", TRUE, NULL);
    g_signal_connect(G_OBJECT(sink), "new-sample", G_CALLBACK(new_sample), &info);

    GstClock *clock{gst_system_clock_obtain()};
    g_object_set(clock, "clock-type", GST_CLOCK_TYPE_MONOTONIC, NULL);
    gst_pipeline_use_clock(GST_PIPELINE_CAST(pipeline), clock);
    gst_object_unref(clock);

    /* Start playing */
    ret = gst_element_set_state(pipeline, GST_STATE_PLAYING);

    if (ret == GST_STATE_CHANGE_FAILURE)
    {
        g_printerr("Unable to set the pipeline to the playing state.\n");
        gst_object_unref(pipeline);
        return {};
    }

    /* Wait until error or EOS */
    bus = gst_element_get_bus(pipeline);

    while (!stop)
    {
        msg = gst_bus_timed_pop_filtered(bus, 100 * GST_MSECOND,
                                         GstMessageType(GST_MESSAGE_ERROR | GST_MESSAGE_EOS));
        clock_gettime(CLOCK_MONOTONIC_RAW, &now);

        if (msg != NULL)
        {
            if (GST_MESSAGE_TYPE(msg) == GST_MESSAGE_ERROR)
            {
                g_error("An error occurred! Re-run with the GST_DEBUG=*:WARN environment "
                        "variable set for more details.");
                ret = GstStateChangeReturn(raise(SIGINT));

                if (ret != 0)
                {
                    printf("Error: unable to raise SIGINT signal.\n");
                    exit(0);
                }
            }

            if (GST_MESSAGE_TYPE(msg) == GST_MESSAGE_EOS)
            {
                printf("EOS received, restarting dev: %s...\n", info.device.c_str());

                gst_message_unref(msg);
                gst_object_unref(bus);
                gst_element_set_state(pipeline, GST_STATE_NULL);
                gst_object_unref(pipeline);

                info.restarts++;
                uint64_t diff = (now.tv_sec * 1000 + lround(now.tv_nsec / 1e6)) - info.init_time_ms;

                if (diff > info.max_duration_ms)
                {
                    info.max_duration_ms = diff;
                    msec_to_str(info.max_duration_str, info.max_duration_ms);
                }

                // wait 500 ms
                usleep(500 * 1000);
                goto init;
            }
        }

        /* watchdog */
        uint64_t diff = (now.tv_sec * 1000 + lround(now.tv_nsec / 1e6)) - info.last_frame_ts_ms;

        if (info.last_frame_ts_ms != 0 && (diff > 1000))
        {
            printf("No frames during last second, restarting dev: %s...\n", info.device.c_str());
            gst_message_unref(msg);
            gst_object_unref(bus);
            gst_element_set_state(pipeline, GST_STATE_NULL);
            gst_object_unref(pipeline);
            info.restarts++;
            diff = (now.tv_sec * 1000 + lround(now.tv_nsec / 1e6)) - info.init_time_ms;

            if (info.frames_after_init_count < POOR_INIT_FRAMES_LIMIT)
            {
                ++info.poor_inits_count;
            }
            else
            {
                info.poor_inits_count = 0;
            }

            if (diff > info.max_duration_ms)
            {
                info.max_duration_ms = diff;
                msec_to_str(info.max_duration_str, info.max_duration_ms);
            }

            if (info.poor_inits_count >= MAX_POOR_INITS_BEFORE_LOCK)
            {
                // brake the loop, free resources and wait until process ends
                break;
            }

            goto init;
        }
    }

    /* Free resources */
    gst_message_unref(msg);
    gst_object_unref(bus);
    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(pipeline);

    if (info.poor_inits_count >= MAX_POOR_INITS_BEFORE_LOCK)
    {
        // the stream is really poor, probably device disconnected, lock the current stream permanently
        printf("No stream stablization during 5 consequative attempts. Lock device permanently: %s...\n", info.device.c_str());

        for (;;)
        {
            // lock here and print locked status periodically
            printf("[ %s ] has been locked permanently\n", info.device.c_str());
            usleep(1000 * 1000);
        }
    }

    return 0;
}

static void print_camera_info(camera_info &camera)
{

    printf("parsed configuration:\n");

    printf("\
      device: %s\n\
      width: %d\n\
      height: %d\n\
      pixel_format: %s\n\
      frame_rate: %d\n\
      rotation: %d\n",
           camera.device.c_str(),
           camera.width, camera.height, camera.pixel_format,
           camera.frame_rate, camera.rotation);
}

int main(int argc, char *argv[])
{

    signal(SIGINT, sig_handler);

    camera_info camera;

    if (argc == 4)
    {
        camera.device = "/dev/video" + std::string{argv[1]};
        camera.width = std::stoi(argv[2]);
        camera.height = std::stoi(argv[3]);

        if (verbose)
        {
            print_camera_info(camera);
        }

        gst_init(0, 0);

        run_pipeline(camera);

        gst_deinit();
    }

    return 0;
}
