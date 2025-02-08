#include "./httplib.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <mutex>
#include <thread>

std::mutex frameMutex;
cv::Mat currentFrame;
bool isRunning = true;

// Функция для конвертации матрицы изображения в формат JPEG
std::string matToJpeg(const cv::Mat& frame) {
    std::vector<uchar> buf;
    cv::imencode(".jpg", frame, buf);
    return std::string(buf.begin(), buf.end());
}

// Функция захвата видео с камеры
void captureVideo() {
    cv::VideoCapture cap(0, cv::CAP_V4L2); // Используем V4L2 для захвата
    if (!cap.isOpened()) {
        std::cerr << "Не удалось открыть камеру" << std::endl;
        return;
    }

    while (isRunning) {
        cv::Mat frame;
        cap >> frame; // Считываем новый кадр

        if (frame.empty()) {
            std::cerr << "Не удалось захватить кадр" << std::endl;
            continue;
        }

        // Блокируем мьютекс для безопасной записи текущего кадра
        {
            std::lock_guard<std::mutex> lock(frameMutex);
            currentFrame = frame;
        }
    }

    cap.release();
}

// Функция для запуска веб-сервера
void startWebServer() {
    httplib::Server svr;

    svr.Get("/stream", [](const httplib::Request&, httplib::Response& res) {
        res.set_header("Content-Type", "multipart/x-mixed-replace; boundary=frame");
        
        while (isRunning) {
            cv::Mat frame;

            {
                std::lock_guard<std::mutex> lock(frameMutex);
                frame = currentFrame.clone(); // Создаем копию текущего кадра
            }

            if (!frame.empty()) {
                std::string jpegData = matToJpeg(frame);

                // Формируем ответ с изображением
                res.set_content("Content-Type: image/jpeg\r\nContent-Length: " + std::to_string(jpegData.size()) + "\r\n\r\n" + jpegData + "\r\n--frame\r\n", "multipart/x-mixed-replace; boundary=frame");
            } else {
                res.status = 503; // Если кадр пустой, возвращаем ошибку
                break;
            }

            // Ставим задержку, чтобы избежать перегрузки сервера
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    });

    std::cout << "Сервер запущен: http://<ваш-IP>:8080/stream" << std::endl;
    svr.listen("0.0.0.0", 8080);
}

int main() {
    // Запуск видео захвата в отдельном потоке
    std::thread captureThread(captureVideo);

    // Запуск веб-сервера
    startWebServer();

    captureThread.join(); // Ожидаем завершения потока с видео

    return 0;
}
