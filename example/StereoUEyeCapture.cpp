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
#include "rgbd/camera/IntrinsicsCalibrator.h"
#include "rgbd/camera/ColorCalibrator.h"
#include "rgbd/camera/CameraRotator.h"

using namespace rgbd;

int main(int argc, char *argv[]) {
    if (argc < 5)
        return -1;

    std::shared_ptr<UEye> lraw(new UEye(std::atoi(argv[1]), argv[3], "Left"));
    std::shared_ptr<rgbd::IntrinsicsCalibrator> lcamera(
            new rgbd::IntrinsicsCalibrator(lraw, argv[4]));
    lcamera->start();

    std::shared_ptr<UEye> rraw(new UEye(std::atoi(argv[2]), argv[3], "Right"));
    std::shared_ptr<rgbd::IntrinsicsCalibrator> rcamera(
            new rgbd::IntrinsicsCalibrator(rraw, argv[5]));
    rcamera->start();

    cv::Mat lcolor = cv::Mat::zeros(lcamera->colorSize(), CV_8UC3);
    cv::Mat rcolor = cv::Mat::zeros(rcamera->colorSize(), CV_8UC3);

    cv::namedWindow("Left color", CV_WINDOW_AUTOSIZE | CV_WINDOW_FREERATIO);
    cv::namedWindow("Right color", CV_WINDOW_AUTOSIZE | CV_WINDOW_FREERATIO);
    int key = 0;

    while ((key = cv::waitKey(10)) != 0x1b) {
        lcamera->captureColor(lcolor);
        rcamera->captureColor(rcolor);

//        cv::Mat lc, rc;
//        cv::resize(lcolor, lc, cv::Size(lcolor.cols / 2, lcolor.rows / 2));
//        cv::resize(rcolor, rc, cv::Size(rcolor.cols / 2, rcolor.rows / 2));
//        cv::imshow("Left color", lc);
//        cv::imshow("Right color", rc);
        cv::imshow("Left color", lcolor);
        cv::imshow("Right color", rcolor);
    }

    return 0;
}
