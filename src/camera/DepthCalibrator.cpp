/**
 * @file DepthCameraCalibrator.cpp
 * @author Yutaka Kondo <yutaka.kondo@youtalk.jp>
 * @date Jul 4, 2014
 */

#include "rgbd/camera/DepthCameraCalibrator.h"

namespace rgbd {

DepthCameraCalibrator::DepthCameraCalibrator(std::shared_ptr<DepthCamera> camera) :
        _camera(camera) {
}

DepthCameraCalibrator::~DepthCameraCalibrator() {
}

cv::Size DepthCameraCalibrator::colorSize() const {
    return _camera->colorSize();
}

cv::Size DepthCameraCalibrator::depthSize() const {
    return _camera->depthSize();
}

void DepthCameraCalibrator::start() {
    return _camera->start();
}

void DepthCameraCalibrator::captureRawColor(cv::Mat& buffer) {
    _camera->captureColor(buffer);
}

void DepthCameraCalibrator::captureRawDepth(cv::Mat& buffer) {
    _camera->captureDepth(buffer);
}

void DepthCameraCalibrator::captureRawAmplitude(cv::Mat& buffer) {
    _camera->captureAmplitude(buffer);
}

}
