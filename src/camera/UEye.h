/**
 * @file UEye.h
 * @author Yutaka Kondo <yutaka.kondo@kawadarobot.co.jp>
 * @date Apr 14, 2014
 */

#pragma once

#include "Camera.h"

namespace rgbd {

class UEye: public Camera {
public:
    UEye(const ::size_t& deviceNo,
         const cv::Size& size = cv::Size(640, 480));

    virtual ~UEye();

    cv::Size colorSize() const;

    virtual void start();

    virtual void captureColor(cv::Mat& buffer);

private:
    const cv::Size _size;
};

}
