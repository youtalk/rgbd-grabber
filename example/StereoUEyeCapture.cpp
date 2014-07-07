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
#include "rgbd/camera/ColorCalibrator.h"
#include "rgbd/camera/CameraRotator.h"

using namespace rgbd;

int main(int argc, char *argv[]) {
    if (argc < 5)
        return -1;

    std::shared_ptr<UEye> raw1(new UEye(std::atoi(argv[1]), argv[3], "Left"));
    std::shared_ptr<rgbd::CameraCalibrator> camera1(
            new rgbd::CameraCalibrator(raw1, argv[4]));
    camera1->start();

    std::shared_ptr<UEye> raw2(new UEye(std::atoi(argv[2]), argv[3], "Right"));
    std::shared_ptr<rgbd::CameraCalibrator> camera2(
            new rgbd::CameraCalibrator(raw2, argv[5]));
    camera2->start();

    cv::Mat color1 = cv::Mat::zeros(camera1->colorSize(), CV_8UC3);
    cv::Mat color2 = cv::Mat::zeros(camera2->colorSize(), CV_8UC3);

    cv::namedWindow("Color left", CV_WINDOW_AUTOSIZE | CV_WINDOW_FREERATIO);
    cv::namedWindow("Color right", CV_WINDOW_AUTOSIZE | CV_WINDOW_FREERATIO);
    int key = 0;

    while ((key = cv::waitKey(10)) != 0x1b) {
        camera1->captureColor(color1);
        camera2->captureColor(color2);

//        cv::Mat c1, c2;
//        cv::resize(color1, c1, cv::Size(color1.cols / 2, color1.rows / 2));
//        cv::resize(color2, c2, cv::Size(color2.cols / 2, color2.rows / 2));
//        cv::imshow("Color left", c1);
//        cv::imshow("Color right", c2);
        cv::imshow("Color left", color1);
        cv::imshow("Color right", color2);
    }

    return 0;
}
