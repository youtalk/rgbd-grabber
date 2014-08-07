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
#include <gflags/gflags.h>
#include "rgbd/camera/UEye.h"
#include "rgbd/camera/StereoCamera.h"

using namespace rgbd;

DEFINE_int32(left_id, 0, "left camera id");
DEFINE_int32(right_id, 0, "right camera id");
DEFINE_string(left_conf, "data/ueye-conf.ini", "left camera conf");
DEFINE_string(right_conf, "data/ueye-conf.ini", "right camera conf");
DEFINE_string(intrinsics, "intrinsics.xml", "intrinsics file");
DEFINE_string(extrinsics, "extrinsics.xml", "extrinsics file");

int main(int argc, char *argv[]) {
    gflags::ParseCommandLineFlags(&argc, &argv, true);

    std::shared_ptr<UEye> left(new UEye(FLAGS_left_id, FLAGS_left_conf, "Left"));
    std::shared_ptr<UEye> right(new UEye(FLAGS_right_id, FLAGS_right_conf, "Right"));
    std::shared_ptr<StereoCamera> camera(new StereoCamera(
            left, right, FLAGS_intrinsics, FLAGS_extrinsics));
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
        camera->captureColoredPointCloud(cloud);

        cv::imshow("Left", lcolor);
        cv::imshow("Right", rcolor);
        viewer->showCloud(cloud);
    }

    return 0;
}
