#include <iostream>

#include "opencv2/opencv.hpp"

#include "utils/DMmath.h"

int main(int argc, char** argv) {
    cv::VideoCapture cam(0);

    if (!cam.isOpened()) {
        std::cout << "cannot open uvc camera!!!" << std::endl;
        exit(0);
    }

    cam.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    cam.set(cv::CAP_PROP_FPS, 30);
    std::cout << "exposure time: " << cam.get(cv::CAP_PROP_EXPOSURE) << std::endl;
    cv::namedWindow("uvc", cv::WINDOW_NORMAL);
    utils::Timer tick;
    while (true) {
        cv::Mat frame;
        tick.Start();
        cam >> frame;
        tick.End();
        // std::cout << "image size: " << frame.size() << std::endl;
        cv::imshow("uvc", frame);
        cv::waitKey(0);
    }
}