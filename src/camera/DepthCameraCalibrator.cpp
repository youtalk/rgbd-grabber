/**
 * @file DepthCameraCalibrator.cpp
 * @author Yutaka Kondo <yutaka.kondo@youtalk.jp>
 * @date Jun 18, 2014
 */

#include "rgbd/camera/DepthCameraCalibrator.h"

namespace rgbd {

DepthCameraCalibrator::DepthCameraCalibrator(std::shared_ptr<DepthCamera> camera,
                                   const std::string& file):
        _camera(camera) {
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
    cv::FileStorage fs(file , CV_STORAGE_READ);

    if (fs.isOpened()) {
        fs["cameraMatrix"] >> cameraMatrix;
        fs["distCoeffs"] >> distCoeffs;

        std::cerr << "DepthCameraCalibrator: cameraMatrix = " << std::endl;
        std::cout << cameraMatrix << std::endl;
        std::cerr << "DepthCameraCalibrator: distCoeffs = " << std::endl;
        std::cout << distCoeffs << std::endl;

        cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(), cameraMatrix,
                                    camera->colorSize(), CV_16SC2,
                                    _rectifyMaps[0], _rectifyMaps[1]);

        std::cout << "DepthCameraCalibrator: undistorted" << std::endl;
        fs.release();
    } else {
        std::cerr << "DepthCameraCalibrator: cannot open file" << std::endl;
        std::exit(-1);
    }
}

DepthCameraCalibrator::~DepthCameraCalibrator() {
}

cv::Size DepthCameraCalibrator::colorSize() const {
    return _camera->colorSize();
}

void DepthCameraCalibrator::start() {
    _camera->start();
}

void DepthCameraCalibrator::captureColor(cv::Mat& buffer) {
    _camera->captureColor(buffer);
    cv::remap(buffer, buffer, _rectifyMaps[0], _rectifyMaps[1], CV_INTER_LINEAR);
}

void DepthCameraCalibrator::captureRawColor(cv::Mat& buffer) {
    _camera->captureColor(buffer);
}

void DepthCameraCalibrator::captureDepth(cv::Mat& buffer) {
}

void DepthCameraCalibrator::captureRawDepth(cv::Mat& buffer) {
    _camera->captureDepth(buffer);
}

void DepthCameraCalibrator::captureAmplitude(cv::Mat& buffer) {
}

void DepthCameraCalibrator::captureRawAmplitude(cv::Mat& buffer) {
    _camera->captureAmplitude(buffer);
}

}
