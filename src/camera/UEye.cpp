/**
 * @file UEye.cpp
 * @author Yutaka Kondo <yutaka.kondo@youtalk.jp>
 * @date Apr 14, 2014
 */

#include "rgbd/camera/UEye.h"

namespace rgbd {

UEye::UEye(const uint deviceNo, const std::string& file,
           const std::string& name) :
        _deviceNo(deviceNo),
        _driver(deviceNo, name),
        _size(640, 480) {
    if (_driver.connectCam() != IS_SUCCESS) {
        std::cerr << "UEye: failed to initialize UEye camera" << std::endl;
        std::exit(-1);
    }
    _driver.loadCamConfig(file);
    _size = _driver.getCameraSize();
}

UEye::~UEye() {
    _driver.disconnectCam();
}

cv::Size UEye::colorSize() const {
    return _size;
}

void UEye::start() {
    if (_driver.setFreeRunMode() != IS_SUCCESS) {
        std::cerr << "UEye: failed to start capturing UEye camera" << std::endl;
        std::exit(-1);
    }
}

void UEye::captureColor(cv::Mat& buffer) {
    boost::mutex::scoped_lock lock(_mutex);

    // Wait for up to 10 sec to capture next frame.
    const char* data = _driver.processNextFrame(10000);
    std::memcpy(buffer.data, data,
                3 * sizeof (uchar) * _size.width * _size.height);
}

}
