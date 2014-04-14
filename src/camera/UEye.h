/**
 * @file UEye.h
 * @author Yutaka Kondo <yutaka.kondo@kawadarobot.co.jp>
 * @date Apr 14, 2014
 */

#pragma once

#include <iostream>
#include <boost/thread/thread.hpp>
#include <uEye.h>
#include "Camera.h"

namespace rgbd {

class UEye: public Camera {
public:
    UEye(const ::uint32_t& deviceNo,
         const cv::Size& size = cv::Size(640, 480), double fps = 30);

    virtual ~UEye();

    cv::Size colorSize() const;

    virtual void start();

    virtual void update();

    virtual void captureColor(cv::Mat& buffer);

private:
    const cv::Size _size;

    double _fps;

    uint32_t _camera;

    char *_buffer;
};

}
