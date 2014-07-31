/**
 * @file DistortionCalibrator.cpp
 * @author Yutaka Kondo <yutaka.kondo@youtalk.jp>
 * @date Apr 22, 2014
 */

#include "rgbd/camera/DistortionCalibrator.h"

namespace rgbd {

DistortionCalibrator::DistortionCalibrator(std::shared_ptr<ColorCamera> camera,
                                           const std::string& intrinsics):
        _camera(camera) {
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
    cv::FileStorage fs(intrinsics , CV_STORAGE_READ);

    if (fs.isOpened()) {
        if (!fs["M"].isNone() && !fs["D"].isNone()) {
            fs["M"] >> cameraMatrix;
            fs["D"] >> distCoeffs;
        } else if (!fs["cameraMatrix"].isNone() && !fs["distCoeffs"].isNone()) {
            fs["cameraMatrix"] >> cameraMatrix;
            fs["distCoeffs"] >> distCoeffs;
        }

        std::cerr << "DistortionCalibrator: cameraMatrix = " << std::endl;
        std::cout << cameraMatrix << std::endl;
        std::cerr << "DistortionCalibrator: distCoeffs = " << std::endl;
        std::cout << distCoeffs << std::endl;

        cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(), cameraMatrix,
                                    camera->colorSize(), CV_16SC2,
                                    _rectifyMaps[0], _rectifyMaps[1]);

        std::cout << "DistortionCalibrator: undistorted" << std::endl;
        fs.release();
    } else {
        std::cerr << "DistortionCalibrator: cannot open " << intrinsics << std::endl;
        std::exit(-1);
    }
}

DistortionCalibrator::~DistortionCalibrator() {
}

cv::Size DistortionCalibrator::colorSize() const {
    return _camera->colorSize();
}

void DistortionCalibrator::start() {
    _camera->start();
}

void DistortionCalibrator::captureColor(cv::Mat& buffer) {
    _camera->captureColor(buffer);
    cv::remap(buffer, buffer, _rectifyMaps[0], _rectifyMaps[1], CV_INTER_LINEAR);
}

void DistortionCalibrator::captureRawColor(cv::Mat& buffer) {
    _camera->captureColor(buffer);
}

}
