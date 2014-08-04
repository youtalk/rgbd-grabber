/**
 * @file StereoUEyeCapture.cpp
 * @author Yutaka Kondo <yutaka.kondo@youtalk.jp>
 * @date Jul 10, 2014
 */

#include <memory>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include "rgbd/camera/UEye.h"
#include "rgbd/camera/StereoCamera.h"

using namespace rgbd;

int main(int argc, char *argv[]) {
    if (argc < 5)
        return -1;

    std::shared_ptr<UEye> left(new UEye(std::atoi(argv[1]), argv[3], "Left"));
    std::shared_ptr<UEye> right(new UEye(std::atoi(argv[2]), argv[3], "Right"));
    std::shared_ptr<StereoCamera> camera(new StereoCamera(
            left, right, argv[4], argv[5]));
    camera->start();

    cv::Mat lcolor;
    cv::Mat rcolor;
    std::shared_ptr<pcl::visualization::CloudViewer> viewer(
            new pcl::visualization::CloudViewer("Vertex"));
    ColoredPointCloud::Ptr cloud(new ColoredPointCloud(
            camera->depthSize().width, camera->depthSize().height));

    cv::namedWindow("Left", CV_WINDOW_AUTOSIZE | CV_WINDOW_FREERATIO);
    cv::namedWindow("Right", CV_WINDOW_AUTOSIZE | CV_WINDOW_FREERATIO);
    int key = 0;

    while ((key = cv::waitKey(10)) != 0x1b) {
        camera->captureColorL(lcolor);
        camera->captureColorR(rcolor);
        camera->captureColoredVertex(cloud);

        cv::imshow("Left", lcolor);
        cv::imshow("Right", rcolor);
        viewer->showCloud(cloud);
    }

    return 0;
}
