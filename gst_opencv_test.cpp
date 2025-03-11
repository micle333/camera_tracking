#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <thread>
#include <mutex>
#include <queue>
#include <opencv2/tracking.hpp>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstring>

#define SERIAL_PORT "/dev/ttyUSB0"
int16_t data[4] = {0, 0, -30, 40};

#define WIDTH 1920
#define HEIGHT 1080

std::queue<cv::Mat> frameQueue;  // Очередь для кадров
std::mutex queueMutex;           // Мьютекс для потокобезопасности
bool running = true;             // Флаг для управления потоком

// Функция обработки новых кадров
GstFlowReturn new_sample(GstAppSink *appsink, gpointer user_data) {
    GstSample *sample = gst_app_sink_pull_sample(appsink);
    GstBuffer *buffer = gst_sample_get_buffer(sample);
    GstMapInfo map;

    if (!gst_buffer_map(buffer, &map, GST_MAP_READ)) {
        std::cerr << "Ошибка: Не удалось отобразить буфер." << std::endl;
        gst_sample_unref(sample);
        return GST_FLOW_OK;
    }

    // Преобразуем данные в cv::Mat (BGR)
    cv::Mat frame(cv::Size(WIDTH, HEIGHT), CV_8UC3, (void *)map.data, cv::Mat::AUTO_STEP);

    {
        std::lock_guard<std::mutex> lock(queueMutex);
        frameQueue.push(frame.clone());  // Копируем кадр в очередь
    }

    gst_buffer_unmap(buffer, &map);
    gst_sample_unref(sample);
    return GST_FLOW_OK;
}

// Функция отображения кадров в отдельном потоке
void displayThread() {

    cv::Rect trackedObject;
    cv::Ptr<cv::Tracker> tracker;
    bool isTracking = false;

    cv::Mat frame, hsv, gray, prevGray, filtered, diffFrame;

    // Переменные для диапазона зеленого цвета (настраиваемые с помощью трекбаров)
    int lowerH = 0, upperH = 2; // Диапазон оттенков (H)
    int lowerS = 0, upperS = 2; // Диапазон насыщенности (S)
    int lowerV = 255, upperV = 255; // Диапазон яркости (V)

    int deltaX = 0;
    int deltaY = 0;

    // cv::namedWindow("tracking", cv::WINDOW_NORMAL); 
    // cv::resizeWindow("tracking", 640, 360);
    
    // cv::namedWindow("grey_mask", cv::WINDOW_NORMAL); 
    // cv::resizeWindow("grey_mask", 640, 360);

    ///////////////////////// Подключение Arduino /////////////////////////
    int serial_port = open(SERIAL_PORT, O_RDWR | O_NOCTTY);
    if (serial_port < 0) {
        std::cerr << "Ошибка открытия порта!" << std::endl;
        //return 1;
    }

    struct termios tty;
    if (tcgetattr(serial_port, &tty) != 0) {
        std::cerr << "Ошибка получения параметров порта!" << std::endl;
        //return 1;
    }

    cfsetospeed(&tty, B9600);
    cfsetispeed(&tty, B9600);
    tty.c_cflag = CS8 | CREAD | CLOCAL;
    tty.c_iflag = IGNPAR;

    tty.c_cflag &= ~PARENB; // No parity
    tty.c_cflag &= ~CSTOPB; // 1 stop bit
    tty.c_cflag &= ~CSIZE;  // Clear size bits
    tty.c_cflag |= CS8;   // 8 data bits
    tty.c_lflag &= ~ICANON; // Non-canonical mode (for reading line by line)
    tty.c_lflag &= ~ECHO;  // Disable echo
    tty.c_lflag &= ~ECHOE; // Disable erasure
    tty.c_lflag &= ~ECHONL;// Disable new-line echo
    tty.c_oflag &= ~OPOST; // Prevent output processing

    tcsetattr(serial_port, TCSANOW, &tty);


    int frameCenterX = 1920 / 2;
    int frameCenterY = 1080 / 2;
    int objectX = frameCenterX, objectY = frameCenterY;
    ///////////////////////// Подключение Arduino /////////////////////////

    while (running) {
        cv::Mat frame;
        {
            std::lock_guard<std::mutex> lock(queueMutex);
            if (!frameQueue.empty()) {
                frame = frameQueue.front();
                frameQueue.pop();
            }
        }

        deltaX = 0;
        deltaY = 0;

        if (!frame.empty()) {
            //cv::imshow("Processed Stream", frame);
            

            // Преобразуем изображение в HSV
            cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

            // Фильтрация по цвету с использованием трекбаров
            cv::Scalar lower(lowerH, lowerS, lowerV);
            cv::Scalar upper(upperH, upperS, upperV);
            cv::inRange(hsv, lower, upper, gray);

            //Фильтрация шума
            cv::medianBlur(gray, gray, 5); // Убираем точечные шумы
            cv::morphologyEx(gray, filtered, cv::MORPH_OPEN, cv::Mat(), cv::Point(-1, -1), 2); // Убираем мелкие объекты
            // cv::imshow("grey_mask", filtered);
                    // Поиск движущихся объектов
                    
            if (!prevGray.empty()) {
                cv::absdiff(filtered, prevGray, diffFrame);
                cv::threshold(diffFrame, diffFrame, 25, 255, cv::THRESH_BINARY); // Подсвечиваем движущиеся объекты
                
                std::vector<std::vector<cv::Point>> contours;
                cv::findContours(diffFrame, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

                if (!contours.empty()) {
                    // Берем самый крупный контур
                    auto maxContour = std::max_element(contours.begin(), contours.end(),
                        [](const std::vector<cv::Point> &a, const std::vector<cv::Point> &b) {
                            return cv::contourArea(a) < cv::contourArea(b);
                        });

                    if (cv::contourArea(*maxContour) > 500) { // Игнорируем слишком маленькие объекты
                        cv::Rect boundingBox = cv::boundingRect(*maxContour);
                        objectX = boundingBox.x + boundingBox.width / 2;
                        objectY = boundingBox.y + boundingBox.height / 2;

                        // Рисуем прямоугольник вокруг найденного объекта
                        cv::rectangle(frame, boundingBox, cv::Scalar(0, 255, 0), 2);
                        deltaX = objectX - frameCenterX;
                        deltaY = objectY - frameCenterY;
                    }
                }
            } 

            // Рисуем перекрестие в центре кадра
            // cv::drawMarker(frame, cv::Point(frameCenterX, frameCenterY), cv::Scalar(0, 0, 255), cv::MARKER_CROSS, 20, 2);

            // //Рисуем линию от центра объекта до центра кадра
            // cv::line(frame, cv::Point(objectX, objectY), cv::Point(frameCenterX, frameCenterY), cv::Scalar(255, 0, 0), 2);

            // Вычисляем разницу координат

            std::cout << "ΔX: " << deltaX << ", ΔY: " << deltaY << std::endl;

            // Отправка координат на Arduino
            data[0] = deltaX;
            data[1] = deltaY;

            //cv::imshow("tracking", frame);

            write(serial_port, data, sizeof(data));

            // Обновляем предыдущий кадр
            gray.copyTo(prevGray);
        }


        //cv::waitKey(1);
        // Задержка 1 мс (можно уменьшить)
        // if (cv::waitKey(1) == 27) {
        //     running = false;
        //     break;
        // }
    }
}

int main(int argc, char *argv[]) {    
    gst_init(&argc, &argv);
    std::cout << "Запуск GStreamer..." << std::endl;

    std::string pipeline_str = "v4l2src device=/dev/video0 ! videoconvert ! video/x-raw,format=BGR ! appsink name=sink";

    GError *error = nullptr;
    GstElement *pipeline = gst_parse_launch(pipeline_str.c_str(), &error);
    GstElement *sink = gst_bin_get_by_name(GST_BIN(pipeline), "sink");

    // Устанавливаем обработчик для appsink
    GstAppSinkCallbacks callbacks = {};
    callbacks.new_sample = new_sample;
    gst_app_sink_set_callbacks(GST_APP_SINK(sink), &callbacks, nullptr, nullptr);

    std::cout << "Запуск пайплайна..." << std::endl;
    gst_element_set_state(pipeline, GST_STATE_PLAYING);

    // Запуск потока отображения
    std::thread display(displayThread);

    // Основной цикл обработки сообщений
    GstBus *bus = gst_element_get_bus(pipeline);
    GstMessage *msg;
    while (running) {
        msg = gst_bus_timed_pop_filtered(bus, 100 * GST_MSECOND, (GstMessageType)(GST_MESSAGE_ERROR | GST_MESSAGE_EOS));
        if (msg) {
            GError *err;
            gchar *debug;
            gst_message_parse_error(msg, &err, &debug);
            std::cerr << "Ошибка GStreamer: " << err->message << std::endl;
            g_error_free(err);
            g_free(debug);
            running = false;
            break;
        }
    }

    std::cout << "Завершение работы..." << std::endl;
    running = false;
    display.join();  // Ждем завершения потока отображения

    gst_message_unref(msg);
    gst_object_unref(bus);
    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(pipeline);
    return 0;
}
