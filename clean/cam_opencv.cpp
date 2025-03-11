#include <opencv2/opencv.hpp>
#include <iostream>

int main() {
    
    cv::VideoCapture cap(0);
    if (!cap.isOpened()) {
        std::cerr << "Ошибка: Камера недоступна!" << std::endl;
        return -1;
    }

    cv::Mat frame;

    while (true) {
        cap >> frame;
        if (frame.empty()) break;
        cv::imshow("Camera Feed", frame);
        if (cv::waitKey(1) == 'q') break; 
    }

    cap.release();
    cv::destroyAllWindows();
    return 0;
}
