/**
 * @file DS325Calibrator.cpp
 * @author Yutaka Kondo <yutaka.kondo@youtalk.jp>
 * @date Jun 18, 2014
 */

#include "rgbd/camera/DS325Calibrator.h"

namespace rgbd {

DS325Calibration::DS325Calibration(const std::string& params) :
        _csize(640, 480),
        _dsize(320, 240) {
    loadParameters(params);
}

DS325Calibration::~DS325Calibration() {
}

void DS325Calibration::calibrateColor(cv::Mat& source, cv::Mat& result) {
    cv::Mat temp;

    cv::remap(source, temp, _rectifyMaps[0][0], _rectifyMaps[0][1], CV_INTER_LINEAR);
    cv::resize(temp(validRoi[0]), result, _csize);
}

void DS325Calibration::calibrateDepth(cv::Mat& source, cv::Mat& result) {
    const uint MAX_DEPTH = 1000;
    const uint MIN_DEPTH = 0;

    // I'm not sure why this is neccesary.
    cv::Mat maxDist = cv::Mat::ones(_csize, CV_16U) * MAX_DEPTH;
    cv::Mat minDist = cv::Mat::ones(_csize, CV_16U) * MIN_DEPTH;
    cv::Mat scaled;
    cv::resize(source, scaled, _csize);
    cv::min(scaled, maxDist, scaled);
    scaled -= minDist;

    cv::Mat cropped = scaled(cv::Rect(40, 43, 498, 498 / 4 * 3)); // TODO
    cv::resize(cropped, cropped, _csize);
    cv::Mat temp;
    cv::remap(cropped, temp, _rectifyMaps[1][0], _rectifyMaps[1][1], CV_INTER_LINEAR);
    cv::resize(temp(validRoi[1]), result, _dsize);
}

void DS325Calibration::calibrateAmplitude(cv::Mat& source, cv::Mat& result) {
    cv::Mat scaled;
    cv::resize(source, scaled, _csize);

    cv::Mat cropped = scaled(cv::Rect(40, 43, 498, 498 / 4 * 3)); // TODO
    cv::resize(cropped, cropped, _csize);
    cv::Mat temp;
    cv::remap(cropped, temp, _rectifyMaps[1][0], _rectifyMaps[1][1], CV_INTER_LINEAR);
    cv::resize(temp(validRoi[1]), result, _dsize);
}

void DS325Calibration::loadParameters(const std::string& params) {
    cv::FileStorage fs(params.c_str(), CV_STORAGE_READ);

    if (fs.isOpened()) {
        fs["M1"] >> cameraMatrix[0];
        fs["D1"] >> distCoeffs[0];
        fs["M2"] >> cameraMatrix[1];
        fs["D2"] >> distCoeffs[1];

        fs["R"] >> R;
        fs["T"] >> T;
        fs["R1"] >> R1;
        fs["R2"] >> R2;
        fs["P1"] >> P1;
        fs["P2"] >> P2;
        fs["Q"] >> Q;
        cv::Mat_<int> vroiMat;
        fs["vroi"] >> vroiMat;

        for (int i = 0; i < 2; ++i) {
            validRoi[i].x = vroiMat.at<int>(i, 0);
            validRoi[i].y = vroiMat.at<int>(i, 1);
            validRoi[i].width = vroiMat.at<int>(i, 2);
            validRoi[i].height = vroiMat.at<int>(i, 3);
        }

        fs.release();
    } else {
        std::cerr << "DS325Calibration: can not save the extrinsic parameters\n";
        std::exit(-1);
    }

    cv::initUndistortRectifyMap(cameraMatrix[0], distCoeffs[0], R1, P1,
                                _csize, CV_16SC2, _rectifyMaps[0][0], _rectifyMaps[0][1]);
    cv::initUndistortRectifyMap(cameraMatrix[1], distCoeffs[1], R2, P2,
                                _csize, CV_16SC2, _rectifyMaps[1][0], _rectifyMaps[1][1]);
}

DS325Calibrator::DS325Calibrator(std::shared_ptr<DS325> camera,
                                   const std::string& file):
        _camera(camera),
        _calib(file) {
    if (_camera->colorSize().width != _camera->depthSize().width * 2 ||
        _camera->colorSize().height != _camera->depthSize().height * 2) {
        throw UnsupportedException("color size != depth size * 2");
    }
}

DS325Calibrator::~DS325Calibrator() {
}

cv::Size DS325Calibrator::colorSize() const {
    return _camera->colorSize();
}

cv::Size DS325Calibrator::depthSize() const {
    return _camera->depthSize();
}

void DS325Calibrator::start() {
    _camera->start();
}

void DS325Calibrator::captureColor(cv::Mat& buffer) {
    _camera->captureColor(buffer);
    _calib.calibrateColor(buffer, buffer);
}

void DS325Calibrator::captureRawColor(cv::Mat& buffer) {
    _camera->captureColor(buffer);
}

void DS325Calibrator::captureDepth(cv::Mat& buffer) {
    _camera->captureDepth(buffer);
    _calib.calibrateDepth(buffer, buffer);
}

void DS325Calibrator::captureRawDepth(cv::Mat& buffer) {
    _camera->captureDepth(buffer);
}

void DS325Calibrator::captureAmplitude(cv::Mat& buffer) {
    _camera->captureAmplitude(buffer);
    _calib.calibrateAmplitude(buffer, buffer);
}

void DS325Calibrator::captureRawAmplitude(cv::Mat& buffer) {
    _camera->captureAmplitude(buffer);
}

void DS325Calibrator::captureVertex(PointXYZVector& buffer) {
    _camera->captureVertex(buffer);
}

}
