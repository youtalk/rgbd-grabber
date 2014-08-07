/**
 * @file DS325Calibrator.h
 * @author Yutaka Kondo <yutaka.kondo@youtalk.jp>
 * @date Jun 18, 2014
 */

#pragma once

#include <iostream>
#include <memory>
#include "DepthCalibrator.h"
#include "DS325.h"

namespace rgbd {

class DS325CalibWorker {
public:
    DS325CalibWorker(const std::string& params);

    ~DS325CalibWorker();

    void calibrateColor(cv::Mat &source, cv::Mat &result);

    void calibrateDepth(cv::Mat &source, cv::Mat &result);

    void calibrateAmplitude(cv::Mat &source, cv::Mat &result);

private:
    cv::Size _csize;

    cv::Size _dsize;

    cv::Mat cameraMatrix[2], distCoeffs[2];

    cv::Mat R, T, R1, R2, P1, P2, Q, F;

    cv::Mat _rectifyMaps[2][2];

    cv::Rect validROI[2];

    void loadParameters(const std::string& params);
};

class DS325Calibrator: public DepthCalibrator {
public:
    DS325Calibrator(std::shared_ptr<DS325> camera, const std::string& file);

    virtual ~DS325Calibrator();

    virtual void captureColor(cv::Mat& buffer);

    virtual void captureDepth(cv::Mat& buffer);

    virtual void captureAmplitude(cv::Mat& buffer);

    virtual void capturePointCloud(PointCloud::Ptr buffer);

private:
    DS325CalibWorker _calib;
};

}
