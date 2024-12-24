#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <iostream>

int main() {
    // Открытие видеопотока с камеры
    cv::VideoCapture cap(0); // 0 - первая камера
    if (!cap.isOpened()) {
        std::cerr << "Не удалось открыть камеру" << std::endl;
        return -1;
    } else {
        std::cerr << "Камера открыта" << std::endl;
    }

    cv::Mat frame, gray, prevGray, diff, thresh;
    std::vector<std::vector<cv::Point>> contours;

    bool isTracking = false; // Флаг для трекинга
    cv::Ptr<cv::Tracker> tracker; // Трекер
    cv::Rect trackedObject; // Координаты отслеживаемого объекта

    while (true) {
        // Считывание текущего кадра
        cap >> frame;
        if (frame.empty()) break;

        // Преобразование в градации серого
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        int frameCenterX = frame.cols / 2;
        int frameCenterY = frame.rows / 2;

        if (!prevGray.empty()) {
            // Если трекинг не активен, ищем движущийся объект
            if (!isTracking) {
                // Вычисление разницы между кадрами
                cv::absdiff(gray, prevGray, diff);

                // Бинаризация (пороговая обработка)
                cv::threshold(diff, thresh, 25, 255, cv::THRESH_BINARY);

                // Поиск контуров движущихся объектов
                cv::findContours(thresh, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

                for (const auto& contour : contours) {
                    if (cv::contourArea(contour) > 500) { // Фильтрация по площади
                        // Определяем прямоугольник вокруг объекта
                        trackedObject = cv::boundingRect(contour);

                        // Инициализируем трекер (например, CSRT)
                        tracker = cv::TrackerCSRT::create();
                        tracker->init(frame, trackedObject);
                        isTracking = true;

                        std::cout << "Обнаружен объект для трекинга." << std::endl;
                        break; // Начинаем трекинг первого обнаруженного объекта
                    }
                }
            } else {
                // Обновляем трекинг
                bool success = tracker->update(frame, trackedObject);
                if (success) {
                    // Рисуем прямоугольник вокруг отслеживаемого объекта
                    cv::rectangle(frame, trackedObject, cv::Scalar(0, 255, 0), 2);

                    // Находим центр отслеживаемого объекта
                    cv::Point objectCenter(trackedObject.x + trackedObject.width / 2, 
                                           trackedObject.y + trackedObject.height / 2);

                    // Рисуем линию от центра кадра до объекта
                    cv::line(frame, cv::Point(frameCenterX, frameCenterY), objectCenter, cv::Scalar(0, 0, 255), 2);

                    // Рисуем крестик в центре кадра
                    cv::drawMarker(frame, cv::Point(frameCenterX, frameCenterY), cv::Scalar(255, 0, 0), cv::MARKER_CROSS, 20, 2);

                    // Вычисляем отклонение по осям X и Y
                    int deviationX = objectCenter.x - frameCenterX;
                    int deviationY = objectCenter.y - frameCenterY;

                    // Отображаем отклонение
                    std::string deviationText = "X: " + std::to_string(deviationX) + "  Y: " + std::to_string(deviationY);
                    cv::putText(frame, deviationText, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
                } else {
                    std::cout << "Трекинг потерян." << std::endl;
                    isTracking = false; // Трекинг завершен
                }
            }
        }

        // Отображение текущего кадра
        cv::imshow("Движущиеся объекты", frame);

        // Обновление предыдущего кадра
        gray.copyTo(prevGray);

        // Выход по нажатию клавиши 'q'
        if (cv::waitKey(30) == 'q') break;
    }

    cap.release();
    cv::destroyAllWindows();
    return 0;
}
