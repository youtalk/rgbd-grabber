/**
 * @file StereoUEyeGetCalibrationData.cpp
 * @author Yutaka Kondo <yutaka.kondo@youtalk.jp>
 * @date Jul 17, 2014
 */

#include <vector>
#include <string>
#include <opencv2/opencv.hpp>
#include <gflags/gflags.h>
#include "rgbd/camera/UEye.h"

DEFINE_int32(left_id, 0, "left camera id");
DEFINE_int32(right_id, 0, "right camera id");
DEFINE_string(left_conf, "data/ueye-conf.ini", "left camera conf");
DEFINE_string(right_conf, "data/ueye-conf.ini", "right camera conf");
DEFINE_string(dir, "/tmp/calib", "calibration data directory");
DEFINE_string(suffix, ".png", "file suffix");

void findChessboards(cv::Mat& left, cv::Mat& right) {
    static int imageNum = 0;

    cv::Size patternSize = cv::Size(9, 6);
    std::vector<cv::Point2f> imagePoints[2];

    if (cv::waitKey(10) == 't') {
        if (cv::findChessboardCorners(
                left, patternSize, imagePoints[0],
                CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE) &&
            cv::findChessboardCorners(
                right, patternSize, imagePoints[1],
                CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE)) {
            std::stringstream ls, rs;
            ls << FLAGS_dir << "/left_" << imageNum << FLAGS_suffix;
            rs << FLAGS_dir << "/right_" << imageNum << FLAGS_suffix;

            cv::imwrite(ls.str(), left);
            cv::imwrite(rs.str(), right);

            std::cout << "save: " << ls.str() << ", " << rs.str() << std::endl;
            imageNum++;
            cv::rectangle(left, cv::Point(5, 5),
                          cv::Point(left.cols - 5, left.rows - 5),
                          cv::Scalar(255, 0, 0), 3);
        } else {
            cv::rectangle(left, cv::Point(5, 5),
                          cv::Point(left.cols - 5, left.rows - 5),
                          cv::Scalar(0, 0, 255), 3);
        }
    }


    cv::Mat lc, rc;
    cv::resize(left, lc, cv::Size(left.cols / 2, left.rows / 2));
    cv::resize(right, rc, cv::Size(right.cols / 2, right.rows / 2));
    cv::imshow("Left", lc);
    cv::imshow("Right", rc);

    if (cv::waitKey(1) == 'q')
        std::exit(0);
}

int main(int argc, char* argv[]) {
    gflags::ParseCommandLineFlags(&argc, &argv, true);

    std::shared_ptr<rgbd::ColorCamera> lcamera(
            new rgbd::UEye(FLAGS_left_id, FLAGS_left_conf, "left"));
    lcamera->start();

    std::shared_ptr<rgbd::ColorCamera> rcamera(
            new rgbd::UEye(FLAGS_right_id, FLAGS_right_conf, "right"));
    rcamera->start();

    std::string execstr = "mkdir -p " + FLAGS_dir;
    system(execstr.c_str());

    cv::Mat left(lcamera->colorSize(), CV_8UC3);
    cv::Mat right(rcamera->colorSize(), CV_8UC3);

    cv::namedWindow("Left", CV_WINDOW_AUTOSIZE | CV_WINDOW_FREERATIO);
    cv::namedWindow("Right", CV_WINDOW_AUTOSIZE | CV_WINDOW_FREERATIO);

    while (cv::waitKey(10) != 0x1b) {
        lcamera->captureColor(left);
        rcamera->captureColor(right);

        findChessboards(left, right);
    }

    cv::destroyAllWindows();
    return 0;
}

