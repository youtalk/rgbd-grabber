/**
 * @file Camera.cpp
 * @author Yutaka Kondo <yutaka.kondo@youtalk.jp>
 * @date Apr 14, 2014
 */

#include "rgbd/camera/Camera.h"

namespace rgbd {

Camera::Camera() {
}

Camera::~Camera() {
}

cv::Size Camera::colorSize() const {
    throw new UnsupportedException("colorSize");
}

void Camera::start() {
}

void Camera::captureColor(cv::Mat& buffer) {
    throw new UnsupportedException("captureColor");
}

}
