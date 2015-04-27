/**
 * @file UEyeCalibration.cpp
 * @author Yutaka Kondo <yutaka.kondo@youtalk.jp>
 * @date Apr 23, 2014
 */

#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <iomanip>
#include <memory>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <gflags/gflags.h>
#include "rgbd/camera/UEye.h"

DEFINE_int32(id, 0, "camera id");
DEFINE_string(conf, "data/ueye-conf.ini", "camera configuration");
DEFINE_string(intrinsics, "data/ueye-calib.xml", "camera intrinsic data");
DEFINE_string(dir, "/tmp", "calibration data directory");
DEFINE_string(suffix, ".png", "file suffix");

bool captureBoardImage(cv::Mat& frame, size_t& count) {
    int key = cv::waitKey(10);

    if (key == 'c') {
        std::stringstream stream;
        stream << FLAGS_dir << "/" << count << FLAGS_suffix;
        std::string fileName = stream.str();
        cv::imwrite(fileName, frame);

        std::cout << "UEyeCalibration: saved " << fileName << std::endl;
        count++;
    } else if (key == 'q') {
        std::cout << "UEyeCalibration: finished capture" << std::endl;
        return false;
    } else if (key == '\x1b') {
        std::exit(0);
    }

    return true;
}

std::vector<cv::Mat> readBoardImage(size_t count) {
    std::vector<cv::Mat> images;

    for (size_t i = 0; i < count; i++) {
        std::stringstream stream;
        stream << FLAGS_dir << "/" << i << FLAGS_suffix;
        std::string fileName = stream.str();
        images.push_back(cv::imread(fileName, 0));

        std::cout << "UEyeCalibration: loaded image " << fileName << std::endl;
    }

    return images;
}

std::vector<std::vector<cv::Point2f>> calculateBoardCorners(
        size_t count, std::vector<cv::Mat>& images, const cv::Size& patternSize) {
    std::vector<std::vector<cv::Point2f>> imagePoints;

    for (size_t i = 0; i < count; i++) {
        std::vector<cv::Point2f> points;

        if (cv::findChessboardCorners(images[i], patternSize, points,
                                      CV_CALIB_CB_ADAPTIVE_THRESH +
                                      CV_CALIB_CB_NORMALIZE_IMAGE +
                                      CV_CALIB_CB_FAST_CHECK)) {
            cv::cornerSubPix(
                    images[i], points, cv::Size(11, 11), cv::Size(-1, -1),
                    cv::TermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, 30, 0.01));
            imagePoints.push_back(points);
            cv::drawChessboardCorners(images[i], patternSize,
                                      (cv::Mat) (points), true);
            cv::imshow("Chess board", images[i]);
            cv::waitKey(200);

            std::cout << "UEyeCalibration: found chess board " << i << std::endl;
        } else {
            std::cout << "UEyeCalibration: not found chess board " << i << std::endl;
        }
    }

    return imagePoints;
}

std::vector<std::vector<cv::Point3f>> calculateWorldPoints(
        std::vector<std::vector<cv::Point2f>>& imagePoints,
        const cv::Size& patternSize, float checkSize) {
    std::vector<std::vector<cv::Point3f>> worldPoints(imagePoints.size());

    for (size_t i = 0; i < imagePoints.size(); i++) {
        for (size_t j = 0; j < patternSize.area(); j++) {
            worldPoints[i].push_back(
                    cv::Point3f(static_cast<float>(j % patternSize.width * checkSize),
                                static_cast<float>(j / patternSize.width * checkSize),
                                0.0));
        }
    }

    return worldPoints;
}

int main(int argc, char* argv[]) {
    gflags::ParseCommandLineFlags(&argc, &argv, true);

    std::shared_ptr<rgbd::ColorCamera> camera(new rgbd::UEye(FLAGS_id, FLAGS_conf));
    camera->start();

    size_t index = 0;
    float checkSize = 24.0;
    cv::Mat color = cv::Mat::zeros(camera->colorSize(), CV_8UC3);
    cv::namedWindow("Capture", CV_WINDOW_AUTOSIZE | CV_WINDOW_FREERATIO);

    do {
        camera->captureColor(color);
        cv::imshow("Capture", color);
    } while (captureBoardImage(color, index));

    std::vector<cv::Mat> images = readBoardImage(index);
    const cv::Size patternSize(9, 6);

    cv::destroyWindow("Capture");
    cv::namedWindow("Chess board", CV_WINDOW_AUTOSIZE);

    std::vector<std::vector<cv::Point2f>> imagePoints =
            calculateBoardCorners(index, images, patternSize);
    std::vector<std::vector<cv::Point3f>> worldPoints =
            calculateWorldPoints(imagePoints, patternSize, checkSize);

    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;

    {
        std::cout << "UEyeCalibration: start calibration" << std::endl;

        cv::FileStorage file(FLAGS_intrinsics, cv::FileStorage::READ);
        std::vector<cv::Mat> tvecs;
        std::vector<cv::Mat> rvecs;

        if (file.isOpened()) {
            file["cameraMatrix"] >> cameraMatrix;
            file["distCoeffs"] >> distCoeffs;
            file.release();

            cv::calibrateCamera(worldPoints, imagePoints, images[0].size(),
                                cameraMatrix, distCoeffs, rvecs, tvecs,
                                CV_CALIB_USE_INTRINSIC_GUESS);
        } else {
            cv::calibrateCamera(worldPoints, imagePoints, images[0].size(),
                                cameraMatrix, distCoeffs, rvecs, tvecs);
        }

        std::cout << "UEyeCalibration: finished calibration" << std::endl;
    }
    {
        double apertureWidth = 0, apertureHeight = 0;
        double fovx = 0, fovy = 0;
        double focalLength = 0;
        cv::Point2d principalPoint(0, 0);
        double aspectRatio = 0;

        cv::calibrationMatrixValues(cameraMatrix, images[0].size(),
                                    apertureWidth, apertureHeight, fovx, fovy,
                                    focalLength, principalPoint, aspectRatio);

        cv::FileStorage file(FLAGS_intrinsics, cv::FileStorage::WRITE);
        file << "cameraMatrix" << cameraMatrix;
        file << "distCoeffs" << distCoeffs;
        file << "imageSize" << camera->colorSize();
        file << "apertureWidth" << apertureWidth;
        file << "apertureHeight" << apertureHeight;
        file << "fovx" << fovx;
        file << "fovy" << fovy;
        file << "focalLength" << focalLength;
        file << "principalPoint" << principalPoint;
        file << "aspectRatio" << aspectRatio;
        file.release();

        std::cout << "UEyeCalibration: saved " << FLAGS_intrinsics << std::endl;
    }

    return 0;
}
