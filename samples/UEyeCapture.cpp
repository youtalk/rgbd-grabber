/**
 * @file UEyeCapture.cpp
 * @author Yutaka Kondo <yutaka.kondo@youtalk.jp>
 * @date Apr 20, 2014
 */

#include <memory>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <gflags/gflags.h>
#include "rgbd/camera/UEye.h"
#include "rgbd/camera/DistortionCalibrator.h"
#include "rgbd/camera/ColorCalibrator.h"
#include "rgbd/camera/ColorRotator.h"

using namespace rgbd;

DEFINE_int32(id, 0, "camera id");
DEFINE_string(conf, "data/ueye-conf.ini", "camera configuration");
DEFINE_string(intrinsics, "data/ueye-calib.xml", "camera intrinsic data");

int main(int argc, char *argv[]) {
    gflags::ParseCommandLineFlags(&argc, &argv, true);

    std::shared_ptr<UEye> original(new UEye(FLAGS_id, FLAGS_conf));
    std::shared_ptr<rgbd::DistortionCalibrator> undistorted(
            new rgbd::DistortionCalibrator(original, FLAGS_intrinsics));
    std::shared_ptr<rgbd::ColorCalibrator> camera(
            new rgbd::ColorCalibrator(undistorted));
//    std::shared_ptr<rgbd::ColorRotator> camera(
//            new rgbd::ColorRotator(calibrated, 90));
    camera->start();

    cv::Mat raw = cv::Mat::zeros(camera->colorSize(), CV_8UC3);
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
