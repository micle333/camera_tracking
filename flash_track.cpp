#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstring>

#define SERIAL_PORT "/dev/ttyUSB0"
int16_t data[4] = {0, 0, -30, 40};

// Функция для инициализации камеры
cv::VideoCapture initializeCamera() {
    cv::VideoCapture cap(0); // 0 - первая камера
    if (!cap.isOpened()) {
        std::cerr << "Не удалось открыть камеру" << std::endl;
        exit(-1);
    }
    return cap;
}


int main() {
    cv::VideoCapture cap = initializeCamera();
    cv::Rect trackedObject;
    cv::Ptr<cv::Tracker> tracker;
    bool isTracking = false;

    cv::Mat frame, hsv, gray, prevGray, filtered, diffFrame;

    // Переменные для диапазона зеленого цвета (настраиваемые с помощью трекбаров)
    int lowerH = 0, upperH = 5; // Диапазон оттенков (H)
    int lowerS = 0, upperS = 3; // Диапазон насыщенности (S)
    int lowerV = 255, upperV = 255; // Диапазон яркости (V)

    cv::namedWindow("tracking", cv::WINDOW_NORMAL); 
    cv::resizeWindow("tracking", 640, 360);
    
    cv::namedWindow("grey_mask", cv::WINDOW_NORMAL); 
    cv::resizeWindow("grey_mask", 640, 360);

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
    tcsetattr(serial_port, TCSANOW, &tty);

    cap >> frame;
    int frameCenterX = frame.cols / 2;
    int frameCenterY = frame.rows / 2;
    int objectX = frameCenterX, objectY = frameCenterY;
    ///////////////////////// Подключение Arduino /////////////////////////


    while (true) {
        cap >> frame;
        if (frame.empty()) break;

        // Преобразуем изображение в HSV
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

        // Фильтрация по цвету с использованием трекбаров
        cv::Scalar lower(lowerH, lowerS, lowerV);
        cv::Scalar upper(upperH, upperS, upperV);
        cv::inRange(hsv, lower, upper, gray);

        //Фильтрация шума
        cv::medianBlur(gray, gray, 5); // Убираем точечные шумы
        cv::morphologyEx(gray, filtered, cv::MORPH_OPEN, cv::Mat(), cv::Point(-1, -1), 2); // Убираем мелкие объекты
        cv::imshow("grey_mask", filtered);


        
         // Координаты объекта (по умолчанию — центр кадра)

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
                }
            }
        }

        // Рисуем перекрестие в центре кадра
        cv::drawMarker(frame, cv::Point(frameCenterX, frameCenterY), cv::Scalar(0, 0, 255), cv::MARKER_CROSS, 20, 2);

        // Рисуем линию от центра объекта до центра кадра
        cv::line(frame, cv::Point(objectX, objectY), cv::Point(frameCenterX, frameCenterY), cv::Scalar(255, 0, 0), 2);

        // Вычисляем разницу координат
        int deltaX = objectX - frameCenterX;
        int deltaY = objectY - frameCenterY;
        std::cout << "ΔX: " << deltaX << ", ΔY: " << deltaY << std::endl;

        // Отправка координат на Arduino
        data[0] = deltaX;
        data[1] = deltaY;

        // if (!prevGray.empty()) {
        //     if (!isTracking) {
        //         if (detectAndTrackMovingObject(frame, gray, prevGray, tracker, trackedObject)) {
        //             isTracking = true;
        //         }
        //     } else {
        //         if (!trackObject(tracker, gray, trackedObject)) {
        //             std::cout << "Трекинг потерян." << std::endl;
        //             isTracking = false;
        //         }
        //     }
        // }

        // drawCenterAndDeviation(frame, trackedObject, frameCenterX, frameCenterY);
        cv::imshow("tracking", frame);
        
	    write(serial_port, data, sizeof(data));

        // Обновляем предыдущий кадр
        gray.copyTo(prevGray);

        if (cv::waitKey(30) == 'q') break;
    }

    cap.release();
    close(serial_port);
    cv::destroyAllWindows();
    return 0;
}
