/**
 * @file DS325Capture.cpp
 * @author Yutaka Kondo <yutaka.kondo@youtalk.jp>
 * @date Jul 30, 2013
 */

#include <memory>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include "camera/DS325.h"

using namespace rgbd;

int main(int argc, char *argv[]) {
    if (argc < 2)
        return -1;

    std::shared_ptr<DepthCamera> camera(new DS325(std::atoi(argv[1])));
    camera->start();

    cv::Mat depth = cv::Mat::zeros(camera->depthSize(), CV_16U);
    cv::Mat amplitude = cv::Mat::zeros(camera->depthSize(), CV_16U);
    cv::Mat color = cv::Mat::zeros(camera->colorSize(), CV_8UC3);
    std::shared_ptr<pcl::visualization::CloudViewer> viewer(
            new pcl::visualization::CloudViewer("Vertex"));
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    cloud->points.resize(camera->depthSize().width * camera->depthSize().height);

    cv::namedWindow("Depth", CV_WINDOW_AUTOSIZE | CV_WINDOW_FREERATIO);
    cv::namedWindow("Amplitude", CV_WINDOW_AUTOSIZE | CV_WINDOW_FREERATIO);
    cv::namedWindow("Color", CV_WINDOW_AUTOSIZE | CV_WINDOW_FREERATIO);

    while (cv::waitKey(10) != 0x1b) {
        camera->captureDepth(depth);
        camera->captureAmplitude(amplitude);
        camera->captureColor(color);
        camera->captureVertex(cloud->points);

        cv::Mat d, a;
        depth.convertTo(d, CV_8U, 255.0 / 1000.0);
        amplitude.convertTo(a, CV_8U, 255.0 / 1000.0);

        cv::imshow("Depth", d);
        cv::imshow("Amplitude", a);
        cv::imshow("Color", color);
        viewer->showCloud(cloud);
    }

    return 0;
}
