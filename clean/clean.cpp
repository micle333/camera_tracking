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
#include <fstream>
#include <thread>
#include <atomic>
#include <chrono>
#include <cmath>
#include <algorithm>

using namespace std;

#define GPIO_X_DIR  "338" // Direction X
#define GPIO_X_STEP  "269" // Speed X
#define GPIO_Y_DIR  "425" // Direction Y
#define GPIO_Y_STEP  "396"// Speed Y

#define WIDTH 1920
#define HEIGHT 1080

// Переменные для диапазона зеленого цвета (настраиваемые с помощью трекбаров)
int lowerH = 0, upperH = 2; // Диапазон оттенков (H)
int lowerS = 0, upperS = 2; // Диапазон насыщенности (S)
int lowerV = 255, upperV = 255; // Диапазон яркости (V)

int frameCenterX = WIDTH / 2;
int frameCenterY = HEIGHT / 2;

std::atomic<int> motorSpeedX = 0;
std::atomic<int> motorSpeedY = 0;
std::atomic<int> motorSpeedXcalc = 0;
std::atomic<int> motorSpeedYcalc = 0;
std::vector<float> dataX;        // Массив последних значений для оси X
std::vector<float> dataXcalc;        // Массив последних значений для оси X



class PID {
    private:
        double Kp, Ki, Kd; // Коэффициенты ПИД-регулятора
        double prevError;
        double integral;
    
    public:
        PID(double Kp, double Ki, double Kd) : Kp(Kp), Ki(Ki), Kd(Kd), prevError(0), integral(0) {
        }
    
        double compute(double deltaX, int dt) {
        
            integral += deltaX * (dt/1000);
            double derivative = (deltaX - prevError) / (dt*1000);
            //cout << "PID del: " << deltaX << " prev: " << prevError << "dt: " << dt << "\n";
            prevError = deltaX;
            return Kp * deltaX + Ki * integral + Kd * derivative;
        }
    };

std::vector<int> detectObject(cv::Mat frame){

    vector<int> res = {0 , 0};
    cv::Mat hsv, gray, filtered;
    // Преобразуем изображение в HSV
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

    // Фильтрация по цвету с использованием трекбаров
    cv::Scalar lower(lowerH, lowerS, lowerV);
    cv::Scalar upper(upperH, upperS, upperV);
    cv::inRange(hsv, lower, upper, gray);

    //Фильтрация шума
    // cv::medianBlur(gray, gray, 5); // Убираем точечные шумы
    // cv::morphologyEx(gray, filtered, cv::MORPH_OPEN, cv::Mat(), cv::Point(-1, -1), 2); // Убираем мелкие объекты

    if (!gray.empty()) {
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(gray, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        if (!contours.empty()) {
            // Берем самый крупный контур
            auto maxContour = std::max_element(contours.begin(), contours.end(),
                [](const std::vector<cv::Point> &a, const std::vector<cv::Point> &b) {
                    return cv::contourArea(a) < cv::contourArea(b);
                });

            if (cv::contourArea(*maxContour) > 500) { // Игнорируем слишком маленькие объекты
                cv::Rect boundingBox = cv::boundingRect(*maxContour);
                int objectX = boundingBox.x + boundingBox.width / 2;
                int objectY = boundingBox.y + boundingBox.height / 2;

                // Рисуем прямоугольник вокруг найденного объекта
                //cv::rectangle(frame, boundingBox, cv::Scalar(0, 255, 0), 2);
                res[0] = objectX - frameCenterX;
                res[1] = (objectY - frameCenterY);
            }
        }
    }
    return res;
}

void writeToFile(const std::string &path, const std::string &value) {
    std::ofstream file(path);
    if (file.is_open()) {
        file << value;
        file.close();
    } else {
        std::cerr << "Ошибка: не удалось открыть " << path << std::endl;
    }
}

void exportGPIO(const std::string &pin) {
    writeToFile("/sys/class/gpio/export", pin);
    writeToFile("/sys/class/gpio/gpio" + pin + "/direction", "out");
    usleep(100000); // Задержка для инициализации
}

void setDirection(const std::string &pin, bool direction) {
    writeToFile("/sys/class/gpio/gpio" + pin + "/value", direction ? "1" : "0");
}

void pulseStep(const std::string &pin, int delayMicro) {
    writeToFile("/sys/class/gpio/gpio" + pin + "/value", "1");
    usleep(delayMicro);  // Короткий импульс
    writeToFile("/sys/class/gpio/gpio" + pin + "/value", "0");
    usleep(delayMicro); 
}


void motorControlX() {
    while (true) {
        int speed = std::clamp(motorSpeedXcalc.load(), -1000, 1000);
        if (speed == 0) {
            usleep(10);
            continue;
        }
        bool dir = (speed < 0);
        //int delayMicro = std::abs(8010 - std::abs(speed * 8));
        int delayMicro = 10000 /(std::abs(speed));

        setDirection(GPIO_X_DIR, dir);
        pulseStep(GPIO_X_STEP, delayMicro);
    }
}

void motorControlY() {
    while (true) {
        int speed = std::clamp(motorSpeedYcalc.load(), -1000, 1000);
        if (speed == 0) {
            usleep(10);
            continue;
        } 

        bool dir = (speed < 0);
        //int delayMicro = std::abs(8010 - std::abs(speed * 8));
        int delayMicro = 10000/(std::abs(speed));

        setDirection(GPIO_Y_DIR, dir);
        pulseStep(GPIO_Y_STEP, delayMicro);
    }
}


void graphWindow(){
    cv::Mat plot(600, 1200, CV_8UC3, cv::Scalar(0, 0, 0));
    int width = 1200, height = 600;
    float time = 0;
    float scaleX = 0.2;  // Увеличивает расстояние между точками
    float minY = -1000, maxY = 1000;
    float scaleY = height / (maxY - minY); // Приведение к размеру экрана

    while (true){
        dataX.push_back(motorSpeedX.load());
        dataXcalc.push_back(motorSpeedXcalc.load());


        if (dataX.size() > 5000) dataX.erase(dataX.begin());
        if (dataXcalc.size() > 5000) dataXcalc.erase(dataXcalc.begin());

        plot.setTo(cv::Scalar(0, 0, 0));  // Очищаем экран
        for (size_t i = 1; i < dataX.size(); i++) {
            int x1 = (i - 1) * scaleX;
            int x2 = i * scaleX;
            int y1 = height - (dataX[i - 1] - minY) * scaleY; // Масштабируем Y
            int y2 = height - (dataX[i] - minY) * scaleY;

            int yc1 = height - (dataXcalc[i - 1] - minY) * scaleY; // Масштабируем Y
            int yc2 = height - (dataXcalc[i] - minY) * scaleY;
    
            cv::line(plot, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(0, 255, 0), 2);
            cv::line(plot, cv::Point(x1, yc1), cv::Point(x2, yc2), cv::Scalar(255, 0, 0), 2);
        }
    
        cv::imshow("Serial Plotter", plot);
    
        cv::waitKey(1);
    }

}

int main() {

    // Экспорт GPIO
    exportGPIO(GPIO_X_DIR);
    exportGPIO(GPIO_X_STEP);
    exportGPIO(GPIO_Y_DIR);
    exportGPIO(GPIO_Y_STEP);

    // Запуск потоков для каждого мотора
    std::thread motorThreadX(motorControlX);
    std::thread motorThreadY(motorControlY);

    PID pidX(0.2, 0.21, 0.0171);
    PID pidY(0.2, 0, 3);
    
    cv::VideoCapture cap(0);
    if (!cap.isOpened()) {
        std::cerr << "Ошибка: Камера недоступна!" << std::endl;
        return -1;
    }

    cv::Mat frame;
    // cv::namedWindow("Camera Feed", cv::WINDOW_NORMAL); 
    // cv::resizeWindow("Camera Feed", 640, 360);
    //std::thread PIDdisplay(graphWindow);

    vector<int> target_coord;
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    while (true) {
        cap >> frame;
        if (frame.empty()) break;

        target_coord = detectObject(frame);



        // motorSpeedX.store(target_coord[0]);
        // motorSpeedY.store(target_coord[1]);

        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        int dt = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();

        int calcX = int(pidX.compute(target_coord[0], dt));
        int calcY = int(pidY.compute(target_coord[1], dt));

        motorSpeedXcalc.store(calcX);
        motorSpeedYcalc.store(calcY);

        
        // std::cout << "Diff(ms) = " << dt << std::endl;
        // cout << target_coord[0] << "; " <<  target_coord[1] << "\n";
        // cout << "Calc PID: " << calcX << "; " <<  calcY << "\n";

        begin = std::chrono::steady_clock::now();


        // cv::line(frame, cv::Point(target_coord[0] + frameCenterX, target_coord[1] + frameCenterY), cv::Point(frameCenterX, frameCenterY), cv::Scalar(255, 0, 0), 2);
        // cv::imshow("Camera Feed", frame);
        // if (cv::waitKey(1) == 'q') break; 
    }

    motorSpeedX.store(0);
    motorSpeedY.store(0);
    // Отключаем потоки
    motorThreadX.detach();
    motorThreadY.detach();

    //PIDdisplay.detach();

    cap.release();
    //cv::destroyAllWindows();
    return 0;
}
