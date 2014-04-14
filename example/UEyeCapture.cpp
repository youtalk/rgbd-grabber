/**
 * @file UVCameraCapture.cpp
 * @author Yutaka Kondo <yutaka.kondo@youtalk.jp>
 * @date Feb 10, 2014
 */

#include <memory>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "camera/UEye.h"

using namespace rgbd;

int main(int argc, char *argv[]) {
    if (argc < 2)
        return -1;

    std::shared_ptr<Camera> camera(new UEye(std::atoi(argv[1]), cv::Size(640, 480)));
    camera->start();

    cv::Mat color = cv::Mat::zeros(camera->colorSize(), CV_8UC3);

    cv::namedWindow("Color", CV_WINDOW_AUTOSIZE | CV_WINDOW_FREERATIO);

    while (cv::waitKey(10) != 0x1b) {
        camera->captureColor(color);

        cv::imshow("Color", color);
    }

    return 0;
}
