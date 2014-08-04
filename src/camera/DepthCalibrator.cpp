/**
 * @file DepthCalibrator.cpp
 * @author Yutaka Kondo <yutaka.kondo@youtalk.jp>
 * @date Jul 4, 2014
 */

#include "rgbd/camera/DepthCalibrator.h"

namespace rgbd {

DepthCalibrator::DepthCalibrator(std::shared_ptr<DepthCamera> camera) :
        _camera(camera) {
}

DepthCalibrator::~DepthCalibrator() {
}

cv::Size DepthCalibrator::colorSize() const {
    return _camera->colorSize();
}

cv::Size DepthCalibrator::depthSize() const {
    return _camera->depthSize();
}

void DepthCalibrator::start() {
    return _camera->start();
}

void DepthCalibrator::captureRawColor(cv::Mat& buffer) {
    _camera->captureColor(buffer);
}

void DepthCalibrator::captureRawDepth(cv::Mat& buffer) {
    _camera->captureDepth(buffer);
}

void DepthCalibrator::captureRawAmplitude(cv::Mat& buffer) {
    _camera->captureAmplitude(buffer);
}

}
