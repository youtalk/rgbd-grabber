/**
 * @file DepthRotator.cpp
 * @author Yutaka Kondo <yutaka.kondo@youtalk.jp>
 * @date Jul 31, 2014
 */

#include "rgbd/camera/DepthRotator.h"

namespace rgbd {

DepthRotator::DepthRotator(std::shared_ptr<DepthCamera> camera, int angle) :
        ColorRotator(camera, angle),
        DepthCamera(),
        _camera(camera),
        _dbuffer(cv::Mat::zeros(camera->depthSize(), CV_16U)),
        _abuffer(cv::Mat::zeros(camera->depthSize(), CV_16U)) {
    if (_angle == 0 || _angle == 180 || _angle == -180) {
        _dsize = camera->depthSize();
    } else if (_angle == 90 || _angle == -90) {
        _dsize.width = camera->depthSize().height;
        _dsize.height = camera->depthSize().width;
    } else {
        throw UnsupportedException("Angle must be -90, 0, 90, or 180.");
    }

    double c = std::cos(_angle * M_PI / 180.0);
    double s = std::sin(_angle * M_PI / 180.0);
    _rotation << c, -s,  0,  0,
                 s,  c,  0,  0,
                 0,  0,  1,  0,
                 0,  0,  0,  1;
}

DepthRotator::~DepthRotator() {
}

cv::Size DepthRotator::colorSize() const {
    return ColorRotator::colorSize();
}

cv::Size DepthRotator::depthSize() const {
    return _dsize;
}

void DepthRotator::start() {
    return ColorRotator::start();
}

void DepthRotator::captureColor(cv::Mat& buffer) {
    ColorRotator::captureColor(buffer);
}

void DepthRotator::captureRawColor(cv::Mat& buffer) {
    ColorRotator::captureRawColor(buffer);
}

void DepthRotator::captureDepth(cv::Mat& buffer) {
    _camera->captureDepth(_dbuffer);

    if (_angle == 0) {
        _dbuffer.copyTo(buffer);
    } else if (_angle == 90) {
        cv::transpose(_dbuffer, buffer);
        cv::flip(buffer, buffer, 0);
    } else if (_angle == -90) {
        cv::transpose(_dbuffer, buffer);
        cv::flip(buffer, buffer, 1);
    } else {
        cv::flip(_dbuffer, buffer, -1);
    }
}

void DepthRotator::captureRawDepth(cv::Mat& buffer) {
    _camera->captureDepth(buffer);
}

void DepthRotator::captureAmplitude(cv::Mat& buffer) {
    _camera->captureAmplitude(_abuffer);

    if (_angle == 0) {
        _abuffer.copyTo(buffer);
    } else if (_angle == 90) {
        cv::transpose(_abuffer, buffer);
        cv::flip(buffer, buffer, 0);
    } else if (_angle == -90) {
        cv::transpose(_abuffer, buffer);
        cv::flip(buffer, buffer, 1);
    } else {
        cv::flip(_abuffer, buffer, -1);
    }
}

void DepthRotator::captureRawAmplitude(cv::Mat& buffer) {
    _camera->captureAmplitude(buffer);
}

void DepthRotator::capturePointCloud(PointCloud::Ptr buffer) {
    PointCloud temp1, temp2;

    _camera->capturePointCloud(buffer);
    std::copy(buffer->points.begin(), buffer->points.end(),
              std::back_inserter(temp1.points));
    pcl::transformPointCloud(temp1, temp2, _rotation);
    std::copy(temp2.points.begin(), temp2.points.end(), buffer->points.begin());
}

void DepthRotator::captureRawVertex(PointCloud::Ptr buffer) {
    _camera->capturePointCloud(buffer);
}

void DepthRotator::captureColoredPointCloud(ColoredPointCloud::Ptr buffer) {
    ColoredPointCloud temp1, temp2;

    _camera->captureColoredPointCloud(buffer);
    std::copy(buffer->points.begin(), buffer->points.end(),
              std::back_inserter(temp1.points));
    pcl::transformPointCloud(temp1, temp2, _rotation);
    std::copy(temp2.points.begin(), temp2.points.end(), buffer->points.begin());
}

void DepthRotator::captureRawColoredVertex(ColoredPointCloud::Ptr buffer) {
    _camera->captureColoredPointCloud(buffer);
}

}
