/**
 * @file CameraCalibrator.h
 * @author Yutaka Kondo <yutaka.kondo@kawadarobot.co.jp>
 * @date Apr 22, 2014
 */

#pragma once

#include <iostream>
#include <memory>
#include "Camera.h"

namespace rgbd {

class CameraCalibrator: public Camera {
public:
    CameraCalibrator(std::shared_ptr<Camera> camera, const std::string& file);

    virtual ~CameraCalibrator();

    virtual cv::Size colorSize() const;

    virtual void start();

    virtual void captureColor(cv::Mat& buffer);

private:
    std::shared_ptr<Camera> _camera;

    cv::Mat _rectifyMaps[2];
};

}
