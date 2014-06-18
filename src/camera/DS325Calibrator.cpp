/**
 * @file DS325Calibrator.cpp
 * @author Yutaka Kondo <yutaka.kondo@youtalk.jp>
 * @date Jun 18, 2014
 */

#include "rgbd/camera/DS325Calibrator.h"

namespace rgbd {

DS325Calibrator::DS325Calibrator(std::shared_ptr<DS325> camera,
                                   const std::string& file):
        _camera(camera) {
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
    cv::FileStorage fs(file , CV_STORAGE_READ);

    if (fs.isOpened()) {
        fs["cameraMatrix"] >> cameraMatrix;
        fs["distCoeffs"] >> distCoeffs;

        std::cerr << "DS325Calibrator: cameraMatrix = " << std::endl;
        std::cout << cameraMatrix << std::endl;
        std::cerr << "DS325Calibrator: distCoeffs = " << std::endl;
        std::cout << distCoeffs << std::endl;

        cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(), cameraMatrix,
                                    camera->colorSize(), CV_16SC2,
                                    _rectifyMaps[0], _rectifyMaps[1]);

        std::cout << "DS325Calibrator: undistorted" << std::endl;
        fs.release();
    } else {
        std::cerr << "DS325Calibrator: cannot open file" << std::endl;
        std::exit(-1);
    }
}

DS325Calibrator::~DS325Calibrator() {
}

cv::Size DS325Calibrator::colorSize() const {
    return _camera->colorSize();
}

void DS325Calibrator::start() {
    _camera->start();
}

void DS325Calibrator::captureColor(cv::Mat& buffer) {
    _camera->captureColor(buffer);
    cv::remap(buffer, buffer, _rectifyMaps[0], _rectifyMaps[1], CV_INTER_LINEAR);
}

void DS325Calibrator::captureRawColor(cv::Mat& buffer) {
    _camera->captureColor(buffer);
}

void DS325Calibrator::captureDepth(cv::Mat& buffer) {
}

void DS325Calibrator::captureRawDepth(cv::Mat& buffer) {
    _camera->captureDepth(buffer);
}

void DS325Calibrator::captureAmplitude(cv::Mat& buffer) {
}

void DS325Calibrator::captureRawAmplitude(cv::Mat& buffer) {
    _camera->captureAmplitude(buffer);
}

}
