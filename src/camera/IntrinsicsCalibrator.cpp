/**
 * @file IntrinsicsCalibrator.cpp
 * @author Yutaka Kondo <yutaka.kondo@youtalk.jp>
 * @date Apr 22, 2014
 */

#include "rgbd/camera/IntrinsicsCalibrator.h"

namespace rgbd {

IntrinsicsCalibrator::IntrinsicsCalibrator(std::shared_ptr<Camera> camera,
                                           const std::string& intrinsics):
        _camera(camera) {
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
    cv::FileStorage fs(intrinsics , CV_STORAGE_READ);

    if (fs.isOpened()) {
        if (!fs["cameraMatrix"].isNone() && !fs["distCoeffs"].isNone()) {
            fs["cameraMatrix"] >> cameraMatrix;
            fs["distCoeffs"] >> distCoeffs;
        } else if (!fs["M"].isNone() && !fs["D"].isNone()) {
            fs["M"] >> cameraMatrix;
            fs["D"] >> distCoeffs;
        }

        std::cerr << "CameraCalibrator: cameraMatrix = " << std::endl;
        std::cout << cameraMatrix << std::endl;
        std::cerr << "CameraCalibrator: distCoeffs = " << std::endl;
        std::cout << distCoeffs << std::endl;

        cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(), cameraMatrix,
                                    camera->colorSize(), CV_16SC2,
                                    _rectifyMaps[0], _rectifyMaps[1]);

        std::cout << "CameraCalibrator: undistorted" << std::endl;
        fs.release();
    } else {
        std::cerr << "CameraCalibrator: cannot open file " << intrinsics << std::endl;
        std::exit(-1);
    }
}

IntrinsicsCalibrator::~IntrinsicsCalibrator() {
}

cv::Size IntrinsicsCalibrator::colorSize() const {
    return _camera->colorSize();
}

void IntrinsicsCalibrator::start() {
    _camera->start();
}

void IntrinsicsCalibrator::captureColor(cv::Mat& buffer) {
    _camera->captureColor(buffer);
    cv::remap(buffer, buffer, _rectifyMaps[0], _rectifyMaps[1], CV_INTER_LINEAR);
}

void IntrinsicsCalibrator::captureRawColor(cv::Mat& buffer) {
    _camera->captureColor(buffer);
}

}
