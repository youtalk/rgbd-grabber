/**
 * @file DepthRotator.h
 * @author Yutaka Kondo <yutaka.kondo@youtalk.jp>
 * @date Jul 31, 2014
 */

#pragma once

#include <pcl/common/transforms.h>
#include "rgbd/camera/DepthCamera.h"
#include "rgbd/camera/ColorRotator.h"

namespace rgbd {

class DepthRotator: public DepthCamera, public ColorRotator {
public:
    DepthRotator(std::shared_ptr<DepthCamera> camera, int angle = 0);

    virtual ~DepthRotator();

    virtual cv::Size colorSize() const;

    virtual cv::Size depthSize() const;

    virtual void start();

    virtual void captureColor(cv::Mat& buffer);

    virtual void captureRawColor(cv::Mat& buffer);

    virtual void captureDepth(cv::Mat& buffer);

    virtual void captureRawDepth(cv::Mat& buffer);

    virtual void captureAmplitude(cv::Mat& buffer);

    virtual void captureRawAmplitude(cv::Mat& buffer);

    virtual void captureVertex(PointCloud buffer);

    virtual void captureRawVertex(PointCloud buffer);

    virtual void captureColoredVertex(ColoredPointCloud buffer);

    virtual void captureRawColoredVertex(ColoredPointCloud buffer);

protected:
    std::shared_ptr<DepthCamera> _camera;

    cv::Size _dsize;

    cv::Mat _dbuffer;

    cv::Mat _abuffer;
};

}
