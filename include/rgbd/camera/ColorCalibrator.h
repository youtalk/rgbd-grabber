/**
 * @file ColorCalibrator.h
 * @author Yutaka Kondo <yutaka.kondo@youtalk.jp>
 * @date Jun 23, 2014
 */

#pragma once

#include <iostream>
#include <memory>
#include "ColorCamera.h"

namespace rgbd {

class ColorCalibrator: public ColorCamera {
public:
    ColorCalibrator(std::shared_ptr<ColorCamera> camera);

    virtual ~ColorCalibrator();

    virtual cv::Size colorSize() const;

    virtual void start();

    virtual void setGrayImage(cv::Mat& gray);

    virtual void captureColor(cv::Mat& buffer);

    virtual void captureRawColor(cv::Mat& buffer);

private:
    std::shared_ptr<ColorCamera> _camera;

    double _rscale;

    double _bscale;
};

}
