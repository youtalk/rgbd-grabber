/**
 * @file UVCamera.cpp
 * @author Yutaka Kondo <yutaka.kondo@youtalk.jp>
 * @date Aug 22, 2013
 */

#include "rgbd/camera/UVCamera.h"

namespace rgbd {

UVCamera::UVCamera(size_t deviceNo, const cv::Size& size, double fps) :
        _capture(deviceNo),
        _size(size),
        _usleep(1000000 / fps) {
    _capture.set(CV_CAP_PROP_FRAME_WIDTH, size.width);
    _capture.set(CV_CAP_PROP_FRAME_HEIGHT, size.height);
    if (!_capture.isOpened())
        std::exit(1);

    std::cout << "UVCamera: opened" << std::endl;
}
UVCamera::~UVCamera() {
    _capture.release();
    std::cout << "UVCamera: closed" << std::endl;
}

cv::Size UVCamera::colorSize() const {
    return _size;
}

void UVCamera::start() {
    boost::thread t(boost::bind(&UVCamera::update, this));
}

void UVCamera::update() {
    while (_capture.isOpened()) {
        usleep(_usleep);

        {
            boost::mutex::scoped_lock lock(_mutex);
            _capture >> _buffer;
        }
    }
}

void UVCamera::captureColor(cv::Mat& buffer) {
    boost::mutex::scoped_lock lock(_mutex);
    _buffer.copyTo(buffer);
}

}
