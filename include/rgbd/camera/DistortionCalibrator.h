/**
 * @file DistortionCalibrator.h
 * @author Yutaka Kondo <yutaka.kondo@youtalk.jp>
 * @date Apr 22, 2014
 */

#pragma once

#include <iostream>
#include <memory>
#include "ColorCamera.h"

namespace rgbd {

class DistortionCalibrator: public ColorCamera {
public:
    DistortionCalibrator(std::shared_ptr<ColorCamera> camera,
                         const std::string& intrinsics);

    virtual ~DistortionCalibrator();

    virtual cv::Size colorSize() const;

    virtual void start();

    virtual void captureColor(cv::Mat& buffer);

    virtual void captureRawColor(cv::Mat& buffer);

private:
    std::shared_ptr<ColorCamera> _camera;

    cv::Mat _rectifyMaps[2];
};

}
