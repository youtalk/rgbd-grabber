/**
 * @file DS325.h
 * @author Yutaka Kondo <yutaka.kondo@youtalk.jp>
 * @date Jul 29, 2013
 */

#pragma once

#include <list>
#include <cstdio>
#include <boost/thread/thread.hpp>
#include <DepthSense.hxx>
#include "DepthCamera.h"

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

    virtual void captureVertex(PointXYZVector& buffer);

    /**
     * Copy the latest audio data to the buffer.
     * Note that the buffer must be allocated in advance.
     *
     * @param buffer Returned value of uchar
     */
    virtual void captureAudio(std::vector<uchar>& buffer);

    /**
     * Copy the latest acceleration data to the buffer.
     *
     * @param buffer Returned 3D data of cv::Point3f
     */
    virtual void captureAcceleration(cv::Point3f& acc);

protected:
    const DepthSense::FrameFormat _format;

    const cv::Size _dsize;

    const cv::Size _csize;

    boost::mutex _dmutex;

    boost::mutex _cmutex;

    boost::mutex _amutex_;

    virtual void onNewDepthSample(DepthNode node, DepthNode::NewSampleReceivedData data);

    virtual void onNewColorSample(ColorNode node, ColorNode::NewSampleReceivedData data);

    virtual void onNewAudioSample(AudioNode node, AudioNode::NewSampleReceivedData data);

private:
    DepthNode::NewSampleReceivedData _ddata;

    ColorNode::NewSampleReceivedData _cdata;

    AudioNode::NewSampleReceivedData _adata;

    Context _context;

    DepthNode _depth;

    ColorNode _color;

    AudioNode _audio;

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
