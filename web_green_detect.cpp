#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <iostream>
#include <thread>
#include <vector>
#include <mutex>
#include <condition_variable>
#include <memory>
#include <string>
#include <sstream>

// Библиотека для создания веб-сервера
#include "./httplib.h"

std::mutex frameMutex;
cv::Mat sharedFrame;
bool frameReady = false;
std::condition_variable frameCondition;

void startWebServer() {
    httplib::Server svr;

    svr.Get("/stream", [](const httplib::Request&, httplib::Response& res) {
        std::unique_lock<std::mutex> lock(frameMutex);
        frameCondition.wait(lock, [] { return frameReady; });

        std::vector<uchar> buffer;
        std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 90};
        cv::imencode(".jpg", sharedFrame, buffer, params);

        res.set_content((const char*)buffer.data(), buffer.size(), "image/jpeg");
    });

    std::cout << "Сервер запущен: http://<ваш-IP>:8080/stream" << std::endl;
    svr.listen("0.0.0.0", 8080);
}

int main() {
    cv::VideoCapture cap(0); // 0 - первая камера
    if (!cap.isOpened()) {
        std::cerr << "Не удалось открыть камеру" << std::endl;
        return -1;
    }

    std::thread serverThread(startWebServer);

    cv::Mat frame, hsv, mask, prevGray, diff, thresh;
    std::vector<std::vector<cv::Point>> contours;

    bool isTracking = false;
    cv::Ptr<cv::Tracker> tracker;
    cv::Rect trackedObject;

    while (true) {
        cap >> frame;
        if (frame.empty()) break;

        // Преобразование в HSV
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

        // Фильтрация по цвету (зелёный цвет)
        cv::inRange(hsv, cv::Scalar(35, 50, 50), cv::Scalar(85, 255, 255), mask);

        if (!prevGray.empty()) {
            if (!isTracking) {
                // Вычисление разницы между кадрами
                cv::absdiff(mask, prevGray, diff);

                // Пороговая обработка
                cv::threshold(diff, thresh, 25, 255, cv::THRESH_BINARY);

                // Поиск контуров
                cv::findContours(thresh, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

                for (const auto& contour : contours) {
                    if (cv::contourArea(contour) > 1500) {
                        trackedObject = cv::boundingRect(contour);
                        tracker = cv::TrackerCSRT::create();
                        tracker->init(frame, trackedObject);
                        isTracking = true;

                        std::cout << "Обнаружен зелёный объект для трекинга." << std::endl;
                        break;
                    }
                }
            } else {
                bool success = tracker->update(frame, trackedObject);
                if (success) {
                    cv::rectangle(frame, trackedObject, cv::Scalar(0, 255, 0), 2);
                } else {
                    std::cout << "Трекинг потерян." << std::endl;
                    isTracking = false;
                }
            }
        }

        mask.copyTo(prevGray);

        // Потокобезопасное обновление общего кадра
        {
            std::lock_guard<std::mutex> lock(frameMutex);
            frame.copyTo(sharedFrame);
            frameReady = true;
        }
        frameCondition.notify_all();

        // Показ кадра для отладки
        cv::imshow("Движение зелёных объектов", frame);

        if (cv::waitKey(30) == 'q') break;
    }

    cap.release();
    cv::destroyAllWindows();

    serverThread.join();
    return 0;
}
