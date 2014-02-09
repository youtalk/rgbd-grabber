/**
 * @file PMDNano.h
 * @author Yutaka Kondo <yutaka.kondo@youtalk.jp>
 * @date Jul 9, 2013
 */
#ifndef PMDNANO_H_
#define PMDNANO_H_

#include <string>
#include <cstdlib>
#include <unistd.h>
#include <pmdsdk2.h>
#include <boost/bind.hpp>
#include <boost/thread/thread.hpp>
#include "camera/DepthCamera.h"

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

    virtual void captureVertex(PointXYZRGBVector& buffer);

protected:
    boost::mutex mutex_;

    bool running_;

    size_t width_;

    size_t height_;

    size_t size_;

    PMDHandle handle_;

    PMDDataDescription description_;

    char* source_;

    float* buffer_;

    float* vertexBuffer_;

    void update();

    bool running();

private:
    void open(const std::string& srcPlugin, const std::string& srcParam,
              const std::string& procPlugin, const std::string& procParam);

    void closeByError(const std::string& function);
};

}
#endif /* PMDNANO_H_ */
