/**
 * @file UVCameraCapture.cpp
 * @author Yutaka Kondo <yutaka.kondo@youtalk.jp>
 * @date Feb 10, 2014
 */

#include <memory>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "rgbd/camera/UEye.h"
#include "rgbd/camera/CameraCalibrator.h"

using namespace rgbd;

int main(int argc, char *argv[]) {
    if (argc < 4)
        return -1;

    std::shared_ptr<rgbd::CameraCalibrator> camera(new rgbd::CameraCalibrator(
            std::shared_ptr<rgbd::Camera>(new rgbd::UEye(std::atoi(argv[1]), argv[2])),
            argv[3]));
//    std::shared_ptr<UEye> camera(new UEye(std::atoi(argv[1]), argv[2]));
    camera->start();

    cv::Mat raw = cv::Mat::zeros(camera->colorSize(), CV_8UC3);
    cv::Mat color = cv::Mat::zeros(camera->colorSize(), CV_8UC3);

    cv::namedWindow("Raw", CV_WINDOW_AUTOSIZE | CV_WINDOW_FREERATIO);
    cv::namedWindow("Color", CV_WINDOW_AUTOSIZE | CV_WINDOW_FREERATIO);
    int key = 0;

    while ((key = cv::waitKey(10)) != 0x1b) {
        camera->captureRawColor(raw);
        camera->captureColor(color);

        cv::imshow("Raw", raw);
        cv::imshow("Color", color);
    }

    return 0;
}
