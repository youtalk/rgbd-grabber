/**
 * @file DS325.h
 * @author Yutaka Kondo <yutaka.kondo@kawadarobot.co.jp>
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

namespace krc {

class DS325: public DepthCamera {

public:
    DS325(const size_t deviceNo,
          const DepthSense::FrameFormat frameFormat = FRAME_FORMAT_VGA,
          const size_t movingAverageSize = 10);

    ~DS325();

    cv::Size depthSize() const;

    cv::Size colorSize() const;

    virtual void start();

    void captureDepth(cv::Mat& buffer);

    void captureAmplitude(cv::Mat& buffer);

    void captureColor(cv::Mat& buffer);

    void captureVertex(PointXYZRGBVector& buffer);

    void captureMovingAveragedVertex(PointXYZRGBVector& buffer);

    void captureAudio(std::vector<uchar>& buffer);

    void captureAcceleration(cv::Point3f& acc);

protected:
    const DepthSense::FrameFormat frameFormat_;

    const cv::Size depthSize_;

    const cv::Size colorSize_;

    cv::Mat depthBuffer_;

    cv::Mat amplitudeBuffer_;

    cv::Mat colorBuffer_;

    std::vector<uchar> audioBuffer_;

    PointXYZRGBVector vertexBuffer_;

    std::list<PointXYZRGBVector> mavertices_;

    cv::Point3f acceleration_;

    boost::mutex depthMutex_;

    boost::mutex colorMutex_;

    boost::mutex audioMutex_;

    void onNewDepthSample(DepthNode node, DepthNode::NewSampleReceivedData data);

    void onNewColorSample(ColorNode node, ColorNode::NewSampleReceivedData data);

    void onNewAudioSample(AudioNode node, AudioNode::NewSampleReceivedData data);

private:

    Context context_;

    DepthNode depth_;

    ColorNode color_;

    AudioNode audio_;

    size_t movingAverageSize_;

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
