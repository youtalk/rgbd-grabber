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
#include "../share/Error.h"
#include "Camera.h"

namespace rgbd {

typedef std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ>>
        PointXYZVector;

typedef std::vector<pcl::PointXYZRGB, Eigen::aligned_allocator<pcl::PointXYZRGB>>
        PointXYZRGBVector;

class DepthCamera: public Camera {
public:
    DepthCamera();

    DepthCamera(const std::shared_ptr<Camera> camera);

    virtual ~DepthCamera();

    virtual cv::Size colorSize() const;

    virtual void start();

    virtual void captureColor(cv::Mat& buffer);

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
     * @param buffer Returned cv::Mat of CV_32F
     */
    virtual void captureDepth(cv::Mat& buffer);

    /**
     * Copy the latest amplitude data to the buffer.
     * Note that the buffer must be allocated in advance.
     *
     * @param buffer Returned cv::Mat of CV_32F
     */
    virtual void captureAmplitude(cv::Mat& buffer);

    /**
     * Copy the latest 3D point cloud data to the buffer.
     * Note that the buffer must be allocated in advance.
     *
     * @param buffer Returned vector of pcl::PointXYZ
     */
    virtual void captureVertex(PointXYZVector& buffer);

    /**
     * Copy the latest 3D point cloud data to the buffer.
     * Note that the buffer must be allocated in advance.
     *
     * @param buffer Returned vector of pcl::PointXYZRGB
     */
    virtual void captureColoredVertex(PointXYZRGBVector& buffer);

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
     * @param buffer Returned 3D data of cv::Point3f
     */
    virtual void captureAcceleration(cv::Point3f& acc);

private:
    std::shared_ptr<Camera> _camera;
};

}
