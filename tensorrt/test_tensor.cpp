#include <gst/gst.h>
#include <gst/video/video.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include "gstnvdsmeta.h"
#include <nvbufsurface.h>

// Callback для обработки каждого кадра
static GstPadProbeReturn osd_sink_pad_buffer_probe(GstPad *pad, GstPadProbeInfo *info, gpointer user_data) {
    GstBuffer *buf = (GstBuffer *)info->data;
    NvDsBatchMeta *batch_meta = gst_buffer_get_nvds_batch_meta(buf);
    
    if (!batch_meta) return GST_PAD_PROBE_OK;

    // Перебираем кадры в батче
    for (NvDsMetaList *frame_list = batch_meta->frame_meta_list; frame_list != nullptr; frame_list = frame_list->next) {
        NvDsFrameMeta *frame_meta = (NvDsFrameMeta *)frame_list->data;
        if (!frame_meta) continue;

        // Используем surface_type для доступа
        if (frame_meta->surface_type == NVBUF_COLOR_FORMAT_RGBA) {  // Проверяем тип поверхности (если это соответствует)
            NvBufSurface *surface = (NvBufSurface *)frame_meta->surface_type;  // Возможно, это другой член
            if (!surface) continue;

            // Преобразуем в OpenCV (если нужно)
            cv::Mat frame(surface->surfaceList[0].height, surface->surfaceList[0].width, CV_8UC4, surface->surfaceList[0].mappedAddr.addr[0]);
            cv::cvtColor(frame, frame, cv::COLOR_RGBA2BGR); // Преобразуем в BGR

            // Выводим кадр
            cv::imshow("DeepStream Video", frame);
            cv::waitKey(1);
        }
    }
    return GST_PAD_PROBE_OK;
}
