/**
 * @file StereoCamera.h
 * @author Yutaka Kondo <yutaka.kondo@youtalk.jp>
 * @date Jul 23, 2014
 */

#pragma once

#include <memory>
#include "rgbd/camera/Camera.h"

namespace rgbd {

class StereoCamera: public Camera {
public:
    StereoCamera(std::shared_ptr<Camera> left, std::shared_ptr<Camera> right,
                 const std::string& intrinsics, const std::string& extrinsics);

    virtual ~StereoCamera();

    cv::Size colorSize() const;

    virtual cv::Size colorSizeL() const;

    virtual cv::Size colorSizeR() const;

    virtual void start();

    void captureColor(cv::Mat& buffer);

    virtual void captureColorL(cv::Mat& buffer);

    virtual void captureColorR(cv::Mat& buffer);

protected:
    std::shared_ptr<Camera> _left;

    std::shared_ptr<Camera> _right;
};

}
