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

    virtual bool adjustWhiteBalance();

    virtual bool adjustColorGain(int read, int green, int blue);

private:
    HIDS _deviceNo;

    ueye_cam::UEyeCamDriver _driver;

    cv::Size _size;

    boost::mutex _mutex;
};

}
