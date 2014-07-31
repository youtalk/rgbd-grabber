/**
 * @file ColorCamera.h
 * @author Yutaka Kondo <yutaka.kondo@youtalk.jp>
 * @date Apr 14, 2014
 */

#pragma once

#include <vector>
#include <stdexcept>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "rgbd/common/Error.h"

namespace rgbd {

class ColorCamera {
public:
    ColorCamera();

    virtual ~ColorCamera();

    /**
     * Return the size of color image.
     *
     * @return Size of color image
     */
    virtual cv::Size colorSize() const;

    /**
     * Start the device updating.
     */
    virtual void start();

    /**
     * Copy the latest color data to the buffer.
     * Note that the buffer must be allocated in advance.
     *
     * @param buffer Returned matrix of CV_8UC3
     */
    virtual void captureColor(cv::Mat& buffer);
};

}
