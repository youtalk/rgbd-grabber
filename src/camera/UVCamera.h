/**
 * @file UVCamera.h
 * @author Yutaka Kondo <yutaka.kondo@youtalk.jp>
 * @date Aug 22, 2013
 */

#ifndef UVCAMERA_H_
#define UVCAMERA_H_

#include <cstdlib>
#include <unistd.h>
#include <boost/thread/thread.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "DepthCamera.h"

namespace rgbd {

class UVCamera: public DepthCamera {
public:
    UVCamera(const::size_t& deviceNo, const cv::Size& size = cv::Size(640, 480));

    virtual ~UVCamera();

    cv::Size colorSize() const;

    virtual void start();

    virtual void captureColor(cv::Mat& buffer);

 private:
    const cv::Size _size;

    cv::VideoCapture _capture;

    cv::Mat _buffer;

    boost::mutex _mutex;

    void update();
};

}
#endif /* UVCAMERA_H_ */
