#include <opencv2/opencv.hpp>
#include <iostream>

int main() {

    // Такое пробовал
    // std::string pipeline = 
    //     "nvv4l2camerasrc device=/dev/video0 ! "
    //     "video/x-raw(memory:NVMM),format=(string)UYVY, width=(int)1920, height=(int)1080 ! "
    //     "nvvidconv ! video/x-raw, format=BGRx ! "
    //     "videoconvert ! video/x-raw, format=BGR ! appsink";


    // cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);


    cv::VideoCapture cap(0);
    //cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));

    if (!cap.isOpened()) {
        std::cerr << "Ошибка: Камера недоступна!" << std::endl;
        return -1;
    }

    cv::Mat frame;
    while (true) {
        cap.grab(); 
        cap.retrieve(frame);

        if (frame.empty()) break;

        cv::imshow("Camera Feed", frame);
        
        // Без задержки обновляем окно
        if (cv::pollKey() == 'q') break;
    }

    // Тут через waitKey
    // while (true) {
    //     cap >> frame;
    //     if (frame.empty()) break;

    //     cv::imshow("Camera Feed", frame);
    //     if (cv::waitKey(1) == 'q') break;     // <<------------ Тут через waitKey
    // }

    cap.release();
    cv::destroyAllWindows();
    return 0;
}
