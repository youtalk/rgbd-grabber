/**
 * @file PMDNano.h
 * @author Yutaka Kondo <yutaka.kondo@youtalk.jp>
 * @date Jul 9, 2013
 */

#pragma once

#include <string>
#include <cstdlib>
#include <unistd.h>
#include <pmdsdk2.h>
#include <boost/bind.hpp>
#include <boost/thread/thread.hpp>
#include "DepthCamera.h"

namespace rgbd {

class PMDNano: public DepthCamera {
public:
    PMDNano(const std::string& srcPlugin, const std::string& procPlugin,
            const std::string& srcParam = "", const std::string& procParam = "");

    virtual ~PMDNano();

    virtual cv::Size depthSize() const;

    virtual void start();

    virtual void captureDepth(cv::Mat& buffer);

    virtual void captureAmplitude(cv::Mat& buffer);

    virtual void captureVertex(PointCloud::Ptr buffer);

protected:
    boost::mutex _mutex;

    volatile bool _running;

    size_t _width;

    size_t _height;

    size_t _size;

    PMDHandle _handle;

    PMDDataDescription _description;

    char* _source;

    float* _buffer;

    float* _vbuffer;

    void update();

private:
    void open(const std::string& srcPlugin, const std::string& srcParam,
              const std::string& procPlugin, const std::string& procParam);

    void closeByError(const std::string& function);
};

}
