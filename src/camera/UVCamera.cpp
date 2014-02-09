/**
 * @file UVCamera.cpp
 * @author Yutaka Kondo <yutaka.kondo@youtalk.jp>
 * @date Aug 22, 2013
 */

#include "camera/UVCamera.h"

namespace rgbd {

UVCamera::UVCamera(const size_t& deviceNo, const cv::Size& size) :
        size_(size), capture_(deviceNo) {
    capture_.set(CV_CAP_PROP_FRAME_WIDTH, size.width);
    capture_.set(CV_CAP_PROP_FRAME_HEIGHT, size.height);
    if (!capture_.isOpened())
        std::exit(1);

    std::cout << "UVCamera: opened" << std::endl;
}
UVCamera::~UVCamera() {
}

cv::Size UVCamera::colorSize() const {
    return size_;
}

void UVCamera::start() const {
    boost::thread t(boost::bind(&UVCamera::update, this));
}

void UVCamera::update() {
    while (capture_.isOpened()) {
        usleep(16667); // 60 [Hz]

        {
            boost::mutex::scoped_lock lock(mutex_);
            capture_ >> buffer_;
        }
    }
}

void UVCamera::captureColor(cv::Mat& buffer) {
    boost::mutex::scoped_lock lock(mutex_);
    buffer = buffer_;
}

}
