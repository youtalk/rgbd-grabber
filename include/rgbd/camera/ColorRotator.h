/**
 * @file CameraRotator.h
 * @author Yutaka Kondo <yutaka.kondo@youtalk.jp>
 * @date Jun 23, 2014
 */

#pragma once

#include <iostream>
#include <memory>
#include "ColorCamera.h"

namespace rgbd {

class ColorRotator: public ColorCamera {
public:
    ColorRotator(std::shared_ptr<ColorCamera> camera, int angle = 0);

    virtual ~ColorRotator();

    virtual cv::Size colorSize() const;

    virtual void start();

    virtual void captureColor(cv::Mat& buffer);

    virtual void captureRawColor(cv::Mat& buffer);

protected:
    std::shared_ptr<ColorCamera> _camera;

    cv::Size _csize;

    cv::Mat _cbuffer;

    const int _angle;
};

}
