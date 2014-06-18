/**
 * @file DepthCameraCalibrator.h
 * @author Yutaka Kondo <yutaka.kondo@youtalk.jp>
 * @date Jun 18, 2014
 */

#pragma once

#include <iostream>
#include <memory>
#include "DepthCamera.h"

namespace rgbd {

class DepthCameraCalibrator: public DepthCamera {
public:
    DepthCameraCalibrator(std::shared_ptr<DepthCamera> camera, const std::string& file);

    virtual ~DepthCameraCalibrator();

    virtual cv::Size colorSize() const;

    virtual void start();

    virtual void captureColor(cv::Mat& buffer);

    virtual void captureRawColor(cv::Mat& buffer);

    virtual void captureDepth(cv::Mat& buffer);

    virtual void captureRawDepth(cv::Mat& buffer);

    virtual void captureAmplitude(cv::Mat& buffer);

    virtual void captureRawAmplitude(cv::Mat& buffer);

private:
    std::shared_ptr<DepthCamera> _camera;

    cv::Mat _rectifyMaps[2];
};

}
