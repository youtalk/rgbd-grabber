/**
 * @file UEye.h
 * @author Yutaka Kondo <yutaka.kondo@kawadarobot.co.jp>
 * @date Apr 14, 2014
 */

#pragma once

#include <iostream>
#include <cstring>
#include <uEye.h>
#include "Camera.h"

namespace rgbd {

class UEye: public Camera {
public:
    UEye(const uint deviceNo, const cv::Size& size = cv::Size(752, 480),
         double fps = 60);

    virtual ~UEye();

    cv::Size colorSize() const;

    virtual void start();

    virtual void captureColor(cv::Mat& buffer);

private:
    const cv::Size _size;

    const uint _camera;

    char* _buffer;

    int _bufferId;
};

}
