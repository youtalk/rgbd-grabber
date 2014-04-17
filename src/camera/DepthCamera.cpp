/**
 * @file DepthCamera.cpp
 * @author Yutaka Kondo <yutaka.kondo@youtalk.jp>
 * @date Jul 9, 2013
 */
#include "DepthCamera.h"

namespace rgbd {

DepthCamera::DepthCamera() {
}

DepthCamera::~DepthCamera() {
}

cv::Size DepthCamera::depthSize() const {
    throw new UnsupportedException("depthSize");
}

void DepthCamera::captureDepth(cv::Mat& buffer) {
    throw new UnsupportedException("captureDepth");
}

void DepthCamera::captureAmplitude(cv::Mat& buffer) {
    throw new UnsupportedException("captureAmplitude");
}

void DepthCamera::captureVertex(PointXYZVector& buffer) {
    throw new UnsupportedException("captureVertex");
}

void DepthCamera::captureColoredVertex(PointXYZRGBVector& buffer) {
    throw new UnsupportedException("captureColoredVertex");
}

void DepthCamera::captureAudio(std::vector<uchar>& buffer) {
    throw new UnsupportedException("captureAudio");
}

void DepthCamera::captureAcceleration(cv::Point3f& acc) {
    throw new UnsupportedException("captureAcceleration");
}

}
