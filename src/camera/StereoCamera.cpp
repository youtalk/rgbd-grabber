/**
 * @file StereoCamera.cpp
 * @author Yutaka Kondo <yutaka.kondo@youtalk.jp>
 * @date Jul 23, 2014
 */

#include "rgbd/camera/StereoCamera.h"

namespace rgbd {

StereoCamera::StereoCamera(std::shared_ptr<ColorCamera> left, std::shared_ptr<ColorCamera> right,
                           const std::string& intrinsics, const std::string& extrinsics) :
        _lcamera(left),
        _rcamera(right),
        _lcolor(cv::Mat::zeros(_lcamera->colorSize(), CV_8UC3)),
        _rcolor(cv::Mat::zeros(_rcamera->colorSize(), CV_8UC3)) {
    if (_lcamera->colorSize().width != _rcamera->colorSize().width ||
        _lcamera->colorSize().height != _rcamera->colorSize().height) {
        std::cerr << "StereoCamera: left camera size != right camera size" << std::endl;
        std::exit(-1);
    }

    loadCameraParams(intrinsics, extrinsics);
    setUpStereoParams();
}

StereoCamera::~StereoCamera() {
}

cv::Size StereoCamera::colorSize() const {
    return colorSizeL();
}

cv::Size StereoCamera::colorSizeL() const {
    return _lcamera->colorSize();
}

cv::Size StereoCamera::colorSizeR() const {
    return _rcamera->colorSize();
}

cv::Size StereoCamera::depthSize() const {
    return colorSizeL();
}

void StereoCamera::start() {
    _lcamera->start();
    _rcamera->start();
}

void StereoCamera::captureColor(cv::Mat& buffer) {
    captureColorL(buffer);
}

void StereoCamera::captureColorL(cv::Mat& buffer) {
    _lcamera->captureColor(_lcolor);
    cv::remap(_lcolor, buffer, _map11, _map12, cv::INTER_LINEAR);
    _lcolor = buffer;
}

void StereoCamera::captureColorR(cv::Mat& buffer) {
    _rcamera->captureColor(_rcolor);
    cv::remap(_rcolor, buffer, _map21, _map22, cv::INTER_LINEAR);
    _rcolor = buffer;
}

cv::Mat StereoCamera::reprojectImage() {
    cv::Mat disparity, xyz;

    _sgbm(_lcolor, _rcolor, disparity);
    cv::reprojectImageTo3D(disparity, xyz, _Q, true);

    return xyz;
}

void StereoCamera::capturePointCloud(PointCloud::Ptr buffer) {
    cv::Mat xyz = reprojectImage();

    buffer->points.clear();
    size_t index = 0;
    double zmax = 1.0e4;

    for (int y = 0; y < xyz.rows; y++) {
        for (int x = 0; x < xyz.cols; x++) {
            cv::Vec3f p = xyz.at<cv::Vec3f>(y, x);

            if (fabs(p[2] - zmax) < FLT_EPSILON || fabs(p[2]) >= zmax)
                continue;

            pcl::PointXYZ point;
            point.x = p[0];
            point.y = -p[1];
            point.z = -p[2];

            buffer->points.push_back(point);
        }
    }
}

void StereoCamera::captureColoredPointCloud(ColoredPointCloud::Ptr buffer) {
    captureColorL(_lcolor);
    captureColorR(_rcolor);
    cv::Mat xyz = reprojectImage();

    buffer->points.clear();
    size_t index = 0;
    double zmax = 1.0e4;

    for (int y = 0; y < xyz.rows; y++) {
        for (int x = 0; x < xyz.cols; x++) {
            cv::Vec3f p = xyz.at<cv::Vec3f>(y, x);

            if (fabs(p[2] - zmax) < FLT_EPSILON || fabs(p[2]) >= zmax)
                continue;

            cv::Vec3b bgr = _lcolor.at<cv::Vec3b>(y, x);
            pcl::PointXYZRGB point;
            point.x = p[0];
            point.y = -p[1];
            point.z = -p[2];
            point.b = bgr[0];
            point.g = bgr[1];
            point.r = bgr[2];

            buffer->points.push_back(point);
        }
    }
}

void StereoCamera::loadCameraParams(const std::string& intrinsics,
                                    const std::string& extrinsics) {
    cv::FileStorage fs(intrinsics, CV_STORAGE_READ);
    cv::Mat M1, D1, M2, D2;
    cv::Mat R, T, R1, P1, R2, P2;
    cv::Rect roi1, roi2;

    if (fs.isOpened()) {
        fs["M1"] >> M1;
        fs["D1"] >> D1;
        fs["M2"] >> M2;
        fs["D2"] >> D2;
    } else {
        std::cerr << "StereoCamera: cannot open " << intrinsics << std::endl;
        std::exit(-1);
    }

    fs.open(extrinsics, CV_STORAGE_READ);

    if (fs.isOpened()) {
        fs["R"] >> R;
        fs["T"] >> T;
    } else {
        std::cerr << "StereoCamera: cannot open " << extrinsics << std::endl;
        std::exit(-1);
    }

    cv::Size size = colorSize();
    cv::stereoRectify(M1, D1, M2, D2, size, R, T, R1, R2, P1, P2, _Q,
                      cv::CALIB_ZERO_DISPARITY, -1, size, &roi1, &roi2);
    std::cout << "StereoCamera: stereo rectified" << std::endl;

    cv::initUndistortRectifyMap(M1, D1, R1, P1, size, CV_16SC2, _map11, _map12);
    cv::initUndistortRectifyMap(M2, D2, R2, P2, size, CV_16SC2, _map21, _map22);
    std::cout << "StereoCamera: undistorted" << std::endl;
}

void StereoCamera::setUpStereoParams() {
    _sgbm.preFilterCap = 63;
    _sgbm.SADWindowSize = 3;
    _sgbm.P1 = 8 * 3 * _sgbm.SADWindowSize * _sgbm.SADWindowSize;
    _sgbm.P2 = 32 * 3 * _sgbm.SADWindowSize * _sgbm.SADWindowSize;
    _sgbm.minDisparity = 0;
    _sgbm.numberOfDisparities = 64;
    _sgbm.uniquenessRatio = 10;
    _sgbm.speckleWindowSize = 100;
    _sgbm.speckleRange = 32;
    _sgbm.disp12MaxDiff = 1;
    _sgbm.fullDP = false;
}

}
