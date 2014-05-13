/**
 * @file CameraCalibrator.cpp
 * @author Yutaka Kondo <yutaka.kondo@youtalk.jp>
 * @date Apr 22, 2014
 */

#include "rgbd/camera/CameraCalibrator.h"

namespace rgbd {

CameraCalibrator::CameraCalibrator(std::shared_ptr<Camera> camera,
                                   const std::string& file):
        _camera(camera) {
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
    cv::FileStorage fs(file , CV_STORAGE_READ);

    if (fs.isOpened()) {
        fs["cameraMatrix"] >> cameraMatrix;
        fs["distCoeffs"] >> distCoeffs;

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
        std::cerr << "CameraCalibrator: cannot open file" << std::endl;
        std::exit(-1);
    }
}

CameraCalibrator::~CameraCalibrator() {
}

cv::Size CameraCalibrator::colorSize() const {
    return _camera->colorSize();
}

void CameraCalibrator::start() {
    _camera->start();
}

void CameraCalibrator::captureColor(cv::Mat& buffer) {
    _camera->captureColor(buffer);
    cv::remap(buffer, buffer, _rectifyMaps[0], _rectifyMaps[1], CV_INTER_LINEAR);
}

void CameraCalibrator::captureRawColor(cv::Mat& buffer) {
    _camera->captureColor(buffer);
}

}
