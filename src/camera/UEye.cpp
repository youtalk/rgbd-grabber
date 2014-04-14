/**
 * @file UEye.cpp
 * @author Yutaka Kondo <yutaka.kondo@kawadarobot.co.jp>
 * @date Apr 14, 2014
 */

#include "UEye.h"

namespace rgbd {

UEye::UEye(const ::size_t& deviceNo, const cv::Size& size) :
        _size(size) {
}

UEye::~UEye() {
}

cv::Size UEye::colorSize() const {
    return _size;
}

void UEye::start() {
}

void UEye::captureColor(cv::Mat& buffer) {
}

}
