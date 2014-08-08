/**
 * @file UVCameraCapture.cpp
 * @author Yutaka Kondo <yutaka.kondo@youtalk.jp>
 * @date Feb 10, 2014
 */

#include <memory>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <gflags/gflags.h>
#include "rgbd/camera/UVCamera.h"

using namespace rgbd;

DEFINE_int32(id, 0, "camera id");
DEFINE_int32(width, 640, "image width");
DEFINE_int32(height, 480, "image height");
DEFINE_double(fps, 30.0, "fps");

int main(int argc, char *argv[]) {
    gflags::ParseCommandLineFlags(&argc, &argv, true);

    std::shared_ptr<ColorCamera> camera(new UVCamera(
            FLAGS_id, cv::Size(FLAGS_width, FLAGS_height), FLAGS_fps));
    camera->start();

    cv::Mat color = cv::Mat::zeros(camera->colorSize(), CV_8UC3);

    cv::namedWindow("Color", CV_WINDOW_AUTOSIZE | CV_WINDOW_FREERATIO);

    while (cv::waitKey(10) != 0x1b) {
        camera->captureColor(color);

        cv::imshow("Color", color);
    }

    return 0;
}
