/**
 * @file ColorRotator.cpp
 * @author Yutaka Kondo <yutaka.kondo@youtalk.jp>
 * @date Jun 23, 2014
 */

#include "rgbd/camera/ColorRotator.h"

namespace rgbd {

ColorRotator::ColorRotator(std::shared_ptr<ColorCamera> camera, int angle) :
        _camera(camera),
        _cbuffer(cv::Mat::zeros(camera->colorSize(), CV_8UC3)),
        _angle(angle) {
    if (_angle == 0 || _angle == 180 || _angle == -180) {
        _csize = camera->colorSize();
    } else if (_angle == 90 || _angle == -90) {
        _csize.width = camera->colorSize().height;
        _csize.height = camera->colorSize().width;
    } else {
        throw UnsupportedException("Angle must be -90, 0, 90, or 180.");
    }
}

ColorRotator::~ColorRotator() {
}

cv::Size ColorRotator::colorSize() const {
    return _csize;
}

void ColorRotator::start() {
    _camera->start();
}

void ColorRotator::captureColor(cv::Mat& buffer) {
    _camera->captureColor(_cbuffer);

    if (_angle == 0) {
        _cbuffer.copyTo(buffer);
    } else if (_angle == 90) {
        cv::transpose(_cbuffer, buffer);
        cv::flip(buffer, buffer, 0);
    } else if (_angle == -90) {
        cv::transpose(_cbuffer, buffer);
        cv::flip(buffer, buffer, 1);
    } else {
        cv::flip(_cbuffer, buffer, -1);
    }
}

void ColorRotator::captureRawColor(cv::Mat& buffer) {
    _camera->captureColor(buffer);
}

}
