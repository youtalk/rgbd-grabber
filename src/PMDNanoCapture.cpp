/**
 * @file PMDNanoCapture.cpp
 * @author Yutaka Kondo <yutaka.kondo@youtalk.jp>
 * @date Jul 30, 2013
 */

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include "camera/PMDNano.h"

using namespace rgbd;

int main(int argc, char *argv[]) {
    if (argc < 3)
        return -1;

    std::shared_ptr<DepthCamera> camera(new PMDNano(argv[1], argv[2]));
    camera->start();

    cv::Mat depth = cv::Mat::zeros(camera->depthSize(), CV_16U);
    cv::Mat amplitude = cv::Mat::zeros(camera->depthSize(), CV_16U);
    std::shared_ptr<pcl::visualization::CloudViewer> viewer(
            new pcl::visualization::CloudViewer("Depth"));
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

    cv::namedWindow("Depth", CV_WINDOW_AUTOSIZE | CV_WINDOW_FREERATIO);
    cv::namedWindow("Amplitude", CV_WINDOW_AUTOSIZE | CV_WINDOW_FREERATIO);

    while (cv::waitKey(10) != 0x1b) {
        camera->captureDepth(depth);
        camera->captureAmplitude(amplitude);
        camera->captureVertex(cloud->points);

        for (auto& p: cloud->points) {
            if (p.z > 0.0)
                p.r = p.g = p.b = 255;
        }

        cv::Mat d, a;
        depth.convertTo(d, CV_8U, 255.0 / 1000.0);
        amplitude.convertTo(a, CV_8U, 255.0 / 1000.0);

        cv::imshow("Depth", d);
        cv::imshow("Amplitude", a);
        viewer->showCloud(cloud);
    }

    return 0;
}
