/**
 * @file UVCamera.h
 * @author Yutaka Kondo <yutaka.kondo@kawadarobot.co.jp>
 * @date Aug 22, 2013
 */

#ifndef UVCAMERA_H_
#define UVCAMERA_H_

#include <cstdlib>
#include <unistd.h>
#include <boost/thread/thread.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "camera/DepthCamera.h"

namespace krc {

class UVCamera: public krc::DepthCamera {
public:
    UVCamera(const::size_t& deviceNo, const cv::Size& size);

    virtual ~UVCamera();

    cv::Size colorSize() const;

    void start() const;

    void captureColor(cv::Mat& buffer);

 private:
    const cv::Size size_;

    cv::VideoCapture capture_;

    cv::Mat buffer_;

    boost::mutex mutex_;

    void update();
};

}
#endif /* UVCAMERA_H_ */
