/**
 * @file DS325GetCalibrationData.cpp
 * @author Yutaka Kondo <yutaka.kondo@youtalk.jp>
 * @date Jun 18, 2014
 */

#include <vector>
#include <string>
#include <opencv2/opencv.hpp>
#include <gflags/gflags.h>
#include "rgbd/camera/DS325.h"

DEFINE_int32(id, 0, "camera id");
DEFINE_string(dir, "/tmp/calib", "calibration data directory");
DEFINE_string(depth, "depth_", "depth file prefix");
DEFINE_string(color, "color_", "color file prefix");
DEFINE_string(suffix, ".png", "file suffix");

void findChessboards(cv::Mat& color, cv::Mat& amplitude) {
    #define MAX_DEPTH 1000
    #define MIN_DEPTH 0
    static int imageNum = 0;

    cv::Mat scaled(amplitude.rows * 2, amplitude.cols * 2, CV_16UC1);

    cv::Mat maxDist = cv::Mat::ones(color.rows, color.cols, CV_16UC1) * MAX_DEPTH;
    cv::Mat minDist = cv::Mat::ones(color.rows, color.cols, CV_16UC1) * MIN_DEPTH;
    cv::resize(amplitude, scaled, color.size());
    cv::min(scaled, maxDist, scaled);

    cv::Size patternSize = cv::Size(9, 6);
    std::vector<cv::Point2f> imagePoints[2];

    scaled -= minDist;
    scaled.convertTo(scaled, CV_8UC1, 255.0 / (MAX_DEPTH - MIN_DEPTH));

    if (cv::findChessboardCorners(
            color, patternSize, imagePoints[0],
            CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE) &&
        cv::findChessboardCorners(
                scaled, patternSize, imagePoints[1],
                CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE)) {
        if (cv::waitKey(1) == 't') {
            std::stringstream cs, as;
            cs << FLAGS_dir << "/" << FLAGS_color << imageNum << FLAGS_suffix;
            as << FLAGS_dir << "/" << FLAGS_depth << imageNum << FLAGS_suffix;

            cv::imwrite(cs.str(), color);
            cv::imwrite(as.str(), amplitude);

            std::cout << cs.str() << ", " << as.str() << ": saved" << std::endl;
            imageNum++;
        }

        cv::rectangle(color, cv::Point(5, 5), cv::Point(635, 475),
                      cv::Scalar(255, 0, 0), 3);
    } else {
        cv::rectangle(color, cv::Point(5, 5), cv::Point(635, 475),
                      cv::Scalar(0, 0, 255), 3);
    }

    cv::imshow("color", color);
    cv::imshow("depth", scaled);

    if (cv::waitKey(1) == 'q')
        std::exit(0);
}

int main(int argc, char* argv[]) {
    gflags::ParseCommandLineFlags(&argc, &argv, true);

    std::shared_ptr<rgbd::DepthCamera> camera(
            new rgbd::DS325(FLAGS_id, FRAME_FORMAT_VGA));
    camera->start();

    std::string execstr = "mkdir -p " + FLAGS_dir;
    system(execstr.c_str());

    cv::Mat color(camera->colorSize(), CV_8UC3);
    cv::Mat amplitude(camera->depthSize(), CV_16UC1);

    cv::namedWindow("color", CV_WINDOW_AUTOSIZE | CV_WINDOW_FREERATIO);
    cv::namedWindow("depth", CV_WINDOW_AUTOSIZE | CV_WINDOW_FREERATIO);

    while (cv::waitKey(10) != 0x1b) {
        camera->captureColor(color);
        camera->captureAmplitude(amplitude);

        findChessboards(color, amplitude);
    }

    cv::destroyAllWindows();
    return 0;
}

