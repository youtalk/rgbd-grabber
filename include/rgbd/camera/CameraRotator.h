/**
 * @file CameraRotator.h
 * @author Yutaka Kondo <yutaka.kondo@youtalk.jp>
 * @date Jun 23, 2014
 */

#pragma once

#include <iostream>
#include <memory>
#include "Camera.h"

namespace rgbd {

class CameraRotator: public Camera {
public:
    CameraRotator(std::shared_ptr<Camera> camera, int angle = 0);

    virtual ~CameraRotator();

    virtual cv::Size colorSize() const;

    virtual void start();

    virtual void captureColor(cv::Mat& buffer);

    virtual void captureRawColor(cv::Mat& buffer);

private:
    std::shared_ptr<Camera> _camera;

    cv::Mat _originalBuffer;

    cv::Size _size;

    const int _angle;
};

}
