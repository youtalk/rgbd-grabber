/**
 * @file DS325.h
 * @author Yutaka Kondo <yutaka.kondo@youtalk.jp>
 * @date Jul 29, 2013
 */

#ifndef DS325_H_
#define DS325_H_

#include <list>
#include <cstdio>
#include <boost/thread/thread.hpp>
#include <DepthSense.hxx>
#include "camera/DepthCamera.h"

using namespace DepthSense;

namespace rgbd {

class DS325: public DepthCamera {

public:
    DS325(const size_t deviceNo,
          const DepthSense::FrameFormat frameFormat = FRAME_FORMAT_WXGA_H);

    ~DS325();

    virtual cv::Size depthSize() const;

    virtual cv::Size colorSize() const;

    virtual void start();

    virtual void captureDepth(cv::Mat& buffer);

    virtual void captureAmplitude(cv::Mat& buffer);

    virtual void captureColor(cv::Mat& buffer);

    virtual void captureVertex(PointXYZRGBVector& buffer);

    virtual void captureAudio(std::vector<uchar>& buffer);

    virtual void captureAcceleration(cv::Point3f& acc);

protected:
    const DepthSense::FrameFormat frameFormat_;

    const cv::Size depthSize_;

    const cv::Size colorSize_;

    cv::Mat depthBuffer_;

    cv::Mat amplitudeBuffer_;

    cv::Mat colorBuffer_;

    std::vector<uchar> audioBuffer_;

    PointXYZRGBVector vertexBuffer_;

    cv::Point3f acceleration_;

    boost::mutex depthMutex_;

    boost::mutex colorMutex_;

    boost::mutex audioMutex_;

    virtual void onNewDepthSample(DepthNode node, DepthNode::NewSampleReceivedData data);

    virtual void onNewColorSample(ColorNode node, ColorNode::NewSampleReceivedData data);

    virtual void onNewAudioSample(AudioNode node, AudioNode::NewSampleReceivedData data);

private:

    Context context_;

    DepthNode depth_;

    ColorNode color_;

    AudioNode audio_;

    void update();

    void onDeviceConnected(Context context, Context::DeviceAddedData data);

    void onDeviceDisconnected(Context context, Context::DeviceRemovedData data);

    void onNodeConnected(Device device, Device::NodeAddedData data);

    void onNodeDisconnected(Device device, Device::NodeRemovedData data);

    void configureDepthNode(Node node);

    void configureColorNode(Node node);

    void configureAudioNode(Node node);
};

}
#endif /* DS325_H_ */
