/**
 * @file CameraRotator.cpp
 * @author Yutaka Kondo <yutaka.kondo@youtalk.jp>
 * @date Jun 23, 2014
 */

#include "rgbd/camera/CameraRotator.h"

namespace rgbd {

CameraRotator::CameraRotator(std::shared_ptr<Camera> camera, int angle) :
        _camera(camera),
        _originalBuffer(cv::Mat::zeros(camera->colorSize(), CV_8UC3)),
        _angle(angle) {
    if (_angle == 0 || _angle == 180 || _angle == -180) {
        _size = camera->colorSize();
    } else if (_angle == 90 || _angle == -90) {
        _size.width = camera->colorSize().height;
        _size.height = camera->colorSize().width;
    } else {
        throw UnsupportedException("Angle must be -90, 0, 90, or 180.");
    }
}

CameraRotator::~CameraRotator() {
}

cv::Size CameraRotator::colorSize() const {
    return _size;
}

void CameraRotator::start() {
    _camera->start();
}

void CameraRotator::captureColor(cv::Mat& buffer) {
    _camera->captureColor(_originalBuffer);

    if (_angle == 0) {
        _originalBuffer.copyTo(buffer);
    } else if (_angle == 90) {
        cv::transpose(_originalBuffer, buffer);
        cv::flip(buffer, buffer, 0);
    } else if (_angle == -90) {
        cv::transpose(_originalBuffer, buffer);
        cv::flip(buffer, buffer, 1);
    } else {
        cv::flip(_originalBuffer, buffer, -1);
    }
}

void CameraRotator::captureRawColor(cv::Mat& buffer) {
    _camera->captureColor(buffer);
}

}
