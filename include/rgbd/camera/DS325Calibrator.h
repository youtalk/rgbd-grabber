/**
 * @file DS325Calibrator.h
 * @author Yutaka Kondo <yutaka.kondo@youtalk.jp>
 * @date Jun 18, 2014
 */

#pragma once

#include <iostream>
#include <memory>
#include "DepthCamera.h"
#include "DS325.h"

namespace rgbd {

class DS325Calibrator: public DepthCamera {
public:
    DS325Calibrator(std::shared_ptr<DS325> camera, const std::string& file);

    virtual ~DS325Calibrator();

    virtual cv::Size colorSize() const;

    virtual void start();

    virtual void captureColor(cv::Mat& buffer);

    virtual void captureRawColor(cv::Mat& buffer);

    virtual void captureDepth(cv::Mat& buffer);

    virtual void captureRawDepth(cv::Mat& buffer);

    virtual void captureAmplitude(cv::Mat& buffer);

    virtual void captureRawAmplitude(cv::Mat& buffer);

private:
    std::shared_ptr<DS325> _camera;

    cv::Mat _rectifyMaps[2];
};

}
