/**
 * @file DepthCamera.h
 * @author Yutaka Kondo <yutaka.kondo@youtalk.jp>
 * @date Jul 9, 2013
 */

#pragma once

#include <vector>
#include <stdexcept>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <pcl/point_types.h>
#include <Eigen/StdVector>
#include "Camera.h"
#include "util/Error.h"

namespace rgbd {

typedef std::vector<pcl::PointXYZRGB, Eigen::aligned_allocator<pcl::PointXYZRGB>>
        PointXYZRGBVector;

class DepthCamera: public Camera {
public:
    DepthCamera();

    virtual ~DepthCamera();

    /**
     * Return the size of depth image.
     *
     * @return Size of depth image
     */
    virtual cv::Size depthSize() const;

    /**
     * Copy the latest depth data to the buffer.
     * Note that the buffer must be allocated in advance.
     *
     * @param buffer Returned matrix of CV_32F
     */
    virtual void captureDepth(cv::Mat& buffer);

    /**
     * Copy the latest amplitude data to the buffer.
     * Note that the buffer must be allocated in advance.
     *
     * @param buffer Returned matrix of CV_32F
     */
    virtual void captureAmplitude(cv::Mat& buffer);

    /**
     * Copy the latest 3D point cloud data to the buffer.
     * Note that the buffer must be allocated in advance.
     *
     * @param buffer Returned vector of PointXYZRGB
     */
    virtual void captureVertex(PointXYZRGBVector& buffer);

    /**
     * Copy the latest audio data to the buffer.
     * Note that the buffer must be allocated in advance.
     *
     * @param buffer Returned value of uchar
     */
    virtual void captureAudio(std::vector<uchar>& buffer);

    /**
     * Copy the latest acceleration data to the buffer.
     *
     * @param buffer Returned 3D data of Point3f
     */
    virtual void captureAcceleration(cv::Point3f& acc);
};

}
