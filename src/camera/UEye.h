/**
 * @file UEye.h
 * @author Yutaka Kondo <yutaka.kondo@kawadarobot.co.jp>
 * @date Apr 14, 2014
 */

#pragma once

#include <iostream>
#include <cstring>
#include <boost/thread.hpp>
#include <uEye.h>
#include "Camera.h"
#include "ueye_cam_driver.hpp"

namespace rgbd {

class UEye: public Camera {
public:
    UEye(const uint deviceNo, const std::string& file);

    virtual ~UEye();

    cv::Size colorSize() const;

    virtual void start();

    virtual void captureColor(cv::Mat& buffer);

private:
    cv::Size _size;

    ueye_cam::UEyeCamDriver _driver;

    boost::mutex _mutex;
};

}
