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
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/StdVector>
#include "../common/Error.h"
#include "ColorCamera.h"

namespace rgbd {

typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloud;

typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr ColoredPointCloud;

class DepthCamera: public ColorCamera {
public:
    DepthCamera();

    DepthCamera(const std::shared_ptr<ColorCamera> camera);

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
     * @param buffer Returned pcl::PointCloud<pcl::PointXYZ>::Ptr
     */
    virtual void captureVertex(PointCloud buffer);

    /**
     * Copy the latest 3D point cloud data to the buffer.
     * Note that the buffer must be allocated in advance.
     *
     * @param buffer Returned pcl::PointCloud<pcl::PointXYZRGB>::Ptr
     */
    virtual void captureColoredVertex(ColoredPointCloud buffer);

private:
    std::shared_ptr<ColorCamera> _camera;
};

}
