/**
 * @file IntrinsicsCalibrator.h
 * @author Yutaka Kondo <yutaka.kondo@youtalk.jp>
 * @date Apr 22, 2014
 */

#pragma once

#include <iostream>
#include <memory>
#include "Camera.h"

namespace rgbd {

class IntrinsicsCalibrator: public Camera {
public:
    IntrinsicsCalibrator(std::shared_ptr<Camera> camera,
                         const std::string& intrinsics);

    virtual ~IntrinsicsCalibrator();

    virtual cv::Size colorSize() const;

    virtual void start();

    virtual void captureColor(cv::Mat& buffer);

    virtual void captureRawColor(cv::Mat& buffer);

private:
    std::shared_ptr<Camera> _camera;

    cv::Mat _rectifyMaps[2];
};

}
