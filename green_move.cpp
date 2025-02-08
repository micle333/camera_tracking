#include <opencv2/opencv.hpp>
#include <iostream>

int main() {
    // Открытие видеопотока с камеры
    cv::VideoCapture cap(0);
    if (!cap.isOpened()) {
        std::cerr << "Не удалось открыть камеру" << std::endl;
        return -1;
    }

    cv::Mat frame, hsv, mask, result;

    // Переменные для диапазона зеленого цвета (настраиваемые с помощью трекбаров)
    int lowerH = 35, upperH = 85; // Диапазон оттенков (H)
    int lowerS = 50, upperS = 255; // Диапазон насыщенности (S)
    int lowerV = 50, upperV = 255; // Диапазон яркости (V)

    // Создаем окно для отображения
    cv::namedWindow("Frame", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("Mask", cv::WINDOW_AUTOSIZE);

    // Создаем трекбары для настройки диапазонов
    cv::createTrackbar("Lower H", "Frame", &lowerH, 179);
    cv::createTrackbar("Upper H", "Frame", &upperH, 179);
    cv::createTrackbar("Lower S", "Frame", &lowerS, 255);
    cv::createTrackbar("Upper S", "Frame", &upperS, 255);
    cv::createTrackbar("Lower V", "Frame", &lowerV, 255);
    cv::createTrackbar("Upper V", "Frame", &upperV, 255);

    while (true) {
        cap >> frame;
        if (frame.empty()) break;

        // Преобразуем изображение в HSV
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

        // Фильтрация по цвету с использованием трекбаров
        cv::Scalar lower(lowerH, lowerS, lowerV);
        cv::Scalar upper(upperH, upperS, upperV);
        cv::inRange(hsv, lower, upper, mask); // Применяем маску

        // Применяем маску к исходному изображению, чтобы выделить только нужные объекты
        cv::bitwise_and(frame, frame, result, mask); // Применяем маску для выделения объектов

        // Отображаем оригинальное изображение, маску и результат
        cv::imshow("Frame", frame);    // Оригинальное изображение
        cv::imshow("Mask", mask);      // Маска
        cv::imshow("Result", result);  // Изображение с примененной маской

        // Выводим на экран текущие значения для каждого компонента
        std::cout << "Lower H: " << lowerH << ", Upper H: " << upperH << std::endl;
        std::cout << "Lower S: " << lowerS << ", Upper S: " << upperS << std::endl;
        std::cout << "Lower V: " << lowerV << ", Upper V: " << upperV << std::endl;

        // Выход по нажатию клавиши 'q'
        if (cv::waitKey(30) == 'q') break;
    }

    cap.release();
    cv::destroyAllWindows();
    return 0;
}
