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

// Функция для обработки каждого кадра
void processFrame(const cv::Mat& gray, cv::Mat& prevGray, cv::Mat& diff, cv::Mat& thresh) {
    // Вычисление разницы между кадрами
    cv::absdiff(gray, prevGray, diff);

    // Бинаризация (пороговая обработка)
    cv::threshold(diff, thresh, 25, 255, cv::THRESH_BINARY);

    // Обновление предыдущего кадра
    gray.copyTo(prevGray);
}

// Функция для трекинга объекта
bool trackObject(cv::Ptr<cv::Tracker>& tracker, cv::Mat& frame, cv::Rect& trackedObject) {
    bool success = tracker->update(frame, trackedObject);
    if (success) {
        cv::rectangle(frame, trackedObject, cv::Scalar(0, 255, 0), 2);
    }
    return success;
}

// Функция для рисования центра и отклонений
void drawCenterAndDeviation(cv::Mat& frame, const cv::Rect& trackedObject, int frameCenterX, int frameCenterY) {
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

    data[0] = deviationX;
    data[1] = deviationY;

    // Отображаем отклонение
    std::string deviationText = "X: " + std::to_string(deviationX) + "  Y: " + std::to_string(deviationY);
    cv::putText(frame, deviationText, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
    
}

// Функция для поиска и трекинга первого объекта
bool detectAndTrackMovingObject(cv::Mat& frame, cv::Mat& gray, cv::Mat& prevGray, cv::Ptr<cv::Tracker>& tracker, cv::Rect& trackedObject) {
    std::vector<std::vector<cv::Point>> contours;
    cv::Mat diff, thresh;

    processFrame(gray, prevGray, diff, thresh);
    cv::findContours(thresh, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    for (const auto& contour : contours) {
        if (cv::contourArea(contour) > 500) { // Фильтрация по площади
            // Определяем прямоугольник вокруг объекта
            trackedObject = cv::boundingRect(contour);

            // Инициализируем трекер (например, CSRT)
            tracker = cv::TrackerCSRT::create();
            tracker->init(frame, trackedObject);

            std::cout << "Обнаружен объект для трекинга." << std::endl;
            return true; // Начинаем трекинг первого обнаруженного объекта
        }
    }
    return false;
}


int main() {
    cv::VideoCapture cap = initializeCamera();
    cv::Mat frame, gray, prevGray;
    cv::Rect trackedObject;
    cv::Ptr<cv::Tracker> tracker;
    bool isTracking = false;
    cv::namedWindow("tracking", cv::WINDOW_NORMAL); 
    cv::resizeWindow("tracking", 640, 360);

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

    
    ///////////////////////// Подключение Arduino /////////////////////////


    while (true) {
        cap >> frame;
        if (frame.empty()) break;

        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        int frameCenterX = frame.cols / 2;
        int frameCenterY = frame.rows / 2;

        if (!prevGray.empty()) {
            if (!isTracking) {
                if (detectAndTrackMovingObject(frame, gray, prevGray, tracker, trackedObject)) {
                    isTracking = true;
                }
            } else {
                if (!trackObject(tracker, frame, trackedObject)) {
                    std::cout << "Трекинг потерян." << std::endl;
                    isTracking = false;
                }
            }
        }

        drawCenterAndDeviation(frame, trackedObject, frameCenterX, frameCenterY);
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
