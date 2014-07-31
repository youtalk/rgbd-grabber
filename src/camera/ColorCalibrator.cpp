/**
 * @file ColorCalibrator.cpp
 * @author Yutaka Kondo <yutaka.kondo@youtalk.jp>
 * @date Jun 23, 2014
 */

#include "rgbd/camera/ColorCalibrator.h"

namespace rgbd {

ColorCalibrator::ColorCalibrator(std::shared_ptr<ColorCamera> camera) :
        _camera(camera),
        _rscale(1.0),
        _bscale(1.0) {
}

ColorCalibrator::~ColorCalibrator() {
}

cv::Size ColorCalibrator::colorSize() const {
    return _camera->colorSize();
}

void ColorCalibrator::start() {
    _camera->start();
}

void ColorCalibrator::setGrayImage(cv::Mat& gray) {
    std::vector<cv::Mat> bgr;
    cv::split(gray, bgr);

    double rsum = 0.0;
    double bsum = 0.0;

    for (size_t y = 0; y < gray.rows; y++) {
        for (size_t x = 0; x < gray.cols; x++) {
            rsum += (double) bgr[1].at<uint8_t>(y, x) / bgr[2].at<uint8_t>(y, x);
            bsum += (double) bgr[1].at<uint8_t>(y, x) / bgr[0].at<uint8_t>(y, x);
        }
    }

    _rscale = rsum / (gray.rows * gray.cols);
    _bscale = bsum / (gray.rows * gray.cols);

    std::cout << "ColorCalibrator: rscale = " << _rscale
              << ", bscale = " << _bscale << std::endl;
}

void ColorCalibrator::captureColor(cv::Mat& buffer) {
    _camera->captureColor(buffer);
    std::vector<cv::Mat> bgr;

    cv::split(buffer, bgr);
    cv::convertScaleAbs(bgr[2], bgr[2], _rscale, 0.0);
    cv::convertScaleAbs(bgr[0], bgr[0], _bscale, 0.0);
    cv::merge(bgr, buffer);
}

void ColorCalibrator::captureRawColor(cv::Mat& buffer) {
    _camera->captureColor(buffer);
}

}
