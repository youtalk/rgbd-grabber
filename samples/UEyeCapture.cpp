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
#include "rgbd/camera/DistortionCalibrator.h"
#include "rgbd/camera/ColorCalibrator.h"
#include "rgbd/camera/CameraRotator.h"

using namespace rgbd;

int main(int argc, char *argv[]) {
    if (argc < 4)
        return -1;

    std::shared_ptr<UEye> original(new UEye(std::atoi(argv[1]), argv[2]));
    std::shared_ptr<rgbd::DistortionCalibrator> calibrated(
            new rgbd::DistortionCalibrator(original, argv[3]));
    std::shared_ptr<rgbd::ColorCalibrator> camera(
            new rgbd::ColorCalibrator(calibrated));
//    std::shared_ptr<rgbd::CameraRotator> camera(
//            new rgbd::CameraRotator(calibrated, 90));
    camera->start();

    cv::Mat raw = cv::Mat::zeros(camera->colorSize().width,
                                 camera->colorSize().height, CV_8UC3);
    cv::Mat color = cv::Mat::zeros(camera->colorSize(), CV_8UC3);

    cv::namedWindow("Raw", CV_WINDOW_AUTOSIZE | CV_WINDOW_FREERATIO);
    cv::namedWindow("Color", CV_WINDOW_AUTOSIZE | CV_WINDOW_FREERATIO);
    int key = 0;

    while ((key = cv::waitKey(10)) != 0x1b) {
        camera->captureRawColor(raw);
        camera->captureColor(color);

        if (key == 'c') {
            cv::Size size = camera->colorSize();
            cv::Mat center = raw(cv::Rect(cv::Point(size.width / 4, size.height / 4),
                                          cv::Size(size.width / 2, size.height / 2)));
            camera->setGrayImage(center);
        }

        cv::imshow("Raw", raw);
        cv::imshow("Color", color);
    }

    return 0;
}
