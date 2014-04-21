/**
 * @file Camera.h
 * @author Yutaka Kondo <yutaka.kondo@kawadarobot.co.jp>
 * @date Apr 14, 2014
 */

#pragma once

#include <vector>
#include <stdexcept>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "../share/Error.h"

namespace rgbd {

class Camera {
public:
    Camera();

    virtual ~Camera();

    /**
     * Return the size of color image.
     *
     * @return Size of color image
     */
    virtual cv::Size colorSize() const;

    /**
     * Start the device updating.
     */
    virtual void start() = 0;

    /**
     * Copy the latest color data to the buffer.
     * Note that the buffer must be allocated in advance.
     *
     * @param buffer Returned matrix of CV_8UC3
     */
    virtual void captureColor(cv::Mat& buffer);
};

}
