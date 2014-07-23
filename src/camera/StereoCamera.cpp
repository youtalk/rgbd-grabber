/**
 * @file StereoCamera.cpp
 * @author Yutaka Kondo <yutaka.kondo@youtalk.jp>
 * @date Jul 23, 2014
 */

#include "rgbd/camera/StereoCamera.h"

namespace rgbd {

StereoCamera::StereoCamera(std::shared_ptr<Camera> left, std::shared_ptr<Camera> right,
                           const std::string& intrinsics, const std::string& extrinsics) :
        _left(left),
        _right(right) {
}

StereoCamera::~StereoCamera() {
}

cv::Size StereoCamera::colorSize() const {
    return colorSizeL();
}

cv::Size StereoCamera::colorSizeL() const {
    return _left->colorSize();
}

cv::Size StereoCamera::colorSizeR() const {
    return _right->colorSize();
}

void StereoCamera::start() {
    _left->start();
    _right->start();
}

void StereoCamera::captureColor(cv::Mat& buffer) {
    captureColorL(buffer);
}

void StereoCamera::captureColorL(cv::Mat& buffer) {
    _left->captureColor(buffer);
}

void StereoCamera::captureColorR(cv::Mat& buffer) {
    _right->captureColor(buffer);
}

}
