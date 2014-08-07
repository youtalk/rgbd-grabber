/**
 * @file DS325.cpp
 * @author Yutaka Kondo <yutaka.kondo@youtalk.jp>
 * @date Jul 29, 2013
 */

#include "rgbd/camera/DS325.h"

namespace rgbd {

DS325::DS325(const size_t deviceNo, const DepthSense::FrameFormat frameFormat) :
        DepthCamera(),
        _format(frameFormat),
        _compression(COMPRESSION_TYPE_MJPEG),
        _dsize(320, 240),
        _context(Context::create("localhost")) {
    if (_format == FRAME_FORMAT_WXGA_H) {
        _csize.width = 1280;
        _csize.height = 720;
    } else if (_format == FRAME_FORMAT_VGA) {
        _csize.width = 640;
        _csize.height = 480;
    } else if (_format == FRAME_FORMAT_QVGA) {
        _csize.width = 320;
        _csize.height = 240;
        _compression = COMPRESSION_TYPE_YUY2;
    } else {
        std::cerr << "DS325: invalid frame format" << std::endl;
        std::exit(-1);
    }

    _context.deviceAddedEvent().connect(this, &DS325::onDeviceConnected);
    _context.deviceRemovedEvent().connect(this, &DS325::onDeviceDisconnected);
    std::vector<Device> devices = _context.getDevices();

    if (deviceNo < devices.size()) {
        devices[deviceNo].nodeAddedEvent().connect(this, &DS325::onNodeConnected);
        devices[deviceNo].nodeRemovedEvent().connect(this, &DS325::onNodeDisconnected);

        for (Node node: devices[deviceNo].getNodes()) {
            if (node.is<DepthNode>() && !_dnode.isSet())
                configureDepthNode(node);
            else if (node.is<ColorNode>() && !_cnode.isSet())
                configureColorNode(node);
            else if (node.is<AudioNode>() && !_anode.isSet())
                configureAudioNode(node);

            _context.registerNode(node);
        }

        std::cout << "DS325: opened" << std::endl;
    } else {
        std::cerr << "DS325: camera " << deviceNo << " cannot open" << std::endl;
        std::exit(-1);
    }
}

DS325::~DS325() {
    if (_dnode.isSet())
        _context.unregisterNode(_dnode);
    if (_cnode.isSet())
        _context.unregisterNode(_cnode);
    if (_anode.isSet())
        _context.unregisterNode(_anode);

    std::cout << "DS325: closed" << std::endl;
}

cv::Size DS325::depthSize() const {
    return _dsize;
}

cv::Size DS325::colorSize() const {
    return _csize;
}

void DS325::update() {
    _context.startNodes();
    _context.run();
    _context.stopNodes();
}

void DS325::start() {
    boost::thread t(boost::bind(&DS325::update, this));
    sleep(3);
}

void DS325::captureDepth(cv::Mat& buffer) {
    boost::mutex::scoped_lock lock(_dmutex);
    std::memcpy(buffer.data, _ddata.depthMap, _ddata.depthMap.size() * 2);
}

void DS325::captureAmplitude(cv::Mat& buffer) {
    boost::mutex::scoped_lock lock(_dmutex);
    std::memcpy(buffer.data, _ddata.confidenceMap, _ddata.confidenceMap.size() * 2);
}

void DS325::captureColor(cv::Mat& buffer) {
    boost::mutex::scoped_lock lock(_cmutex);

    if (_compression == COMPRESSION_TYPE_YUY2)
        buffer = cv::Mat::zeros(_csize, CV_8UC2);

    std::memcpy(buffer.data, _cdata.colorMap, _cdata.colorMap.size());

    if (_compression == COMPRESSION_TYPE_YUY2)
        cv::cvtColor(buffer, buffer, CV_YUV2BGR_YUY2);
}

void DS325::capturePointCloud(PointCloud::Ptr buffer) {
    boost::mutex::scoped_lock lock(_dmutex);
    std::size_t index = 0;

    for (auto& point: buffer->points) {
        auto& f = _ddata.verticesFloatingPoint[index++];
        point.x = f.x;
        point.y = f.y;
        point.z = f.z;
    }
}

void rgbd::DS325::captureColoredPointCloud(ColoredPointCloud::Ptr buffer) {
    cv::Mat color = cv::Mat::zeros(_csize, CV_8UC3);
    captureColor(color);

    boost::mutex::scoped_lock dlock(_dmutex);
    buffer->points.clear();

    for (size_t i = 0; i < _ddata.verticesFloatingPoint.size(); i++) {
        auto& f = _ddata.verticesFloatingPoint[i];
        auto& uv = _ddata.uvMap[i];

        if (uv.u == -FLT_MAX || uv.v == -FLT_MAX)
            continue;

        // TODO: More accurate coloring
        auto& p = color.at<cv::Vec3b>(cvRound(uv.v * _csize.height),
                                      cvRound(uv.u * _csize.width));
        pcl::PointXYZRGB point;
        point.x = f.x;
        point.y = f.y;
        point.z = f.z;
        point.b = p[0];
        point.g = p[1];
        point.r = p[2];

        buffer->points.push_back(point);
    }
}

void DS325::captureAudio(std::vector<uchar>& buffer) {
    boost::mutex::scoped_lock lock(_amutex_);
    buffer.clear();

    for (std::size_t i = 0; i < _adata.audioData.size(); i++)
        buffer.push_back(_adata.audioData[i]);
}

void DS325::captureAcceleration(cv::Point3f& buffer) {
    boost::mutex::scoped_lock lock(_dmutex);

    buffer.x = _ddata.acceleration.x;
    buffer.y = _ddata.acceleration.y;
    buffer.z = _ddata.acceleration.z;
}

void DS325::onDeviceConnected(Context context, Context::DeviceAddedData data) {
}

void DS325::onDeviceDisconnected(Context context, Context::DeviceRemovedData data) {
}

void DS325::onNodeConnected(Device device, Device::NodeAddedData data) {
}

void DS325::onNodeDisconnected(Device device, Device::NodeRemovedData data) {
}

void DS325::onNewDepthSample(DepthNode node, DepthNode::NewSampleReceivedData data) {
    int width, height;
    FrameFormat_toResolution(data.captureConfiguration.frameFormat, &width, &height);

    {
        boost::mutex::scoped_lock lock(_dmutex);
        _ddata = data;
    }
}

void DS325::onNewColorSample(ColorNode node, ColorNode::NewSampleReceivedData data) {
    int width, height;
    FrameFormat_toResolution(data.captureConfiguration.frameFormat, &width, &height);

    {
        boost::mutex::scoped_lock lock(_cmutex);
        _cdata = data;
    }
}

void DS325::onNewAudioSample(AudioNode node, AudioNode::NewSampleReceivedData data) {
    boost::mutex::scoped_lock lock(_amutex_);
    _adata = data;
}

void DS325::configureDepthNode(Node node) {
    _dnode = node.as<DepthNode>();

    DepthNode::Configuration config = _dnode.getConfiguration();
    config.frameFormat = FRAME_FORMAT_QVGA;
    config.framerate = 30;
    config.mode = DepthNode::CAMERA_MODE_CLOSE_MODE;
    config.saturation = false;

    try {
        _context.requestControl(_dnode, 0);
        _dnode.newSampleReceivedEvent().connect(this, &DS325::onNewDepthSample);
        _dnode.setEnableDepthMap(true);
        _dnode.setEnableConfidenceMap(true);
        _dnode.setEnableVerticesFloatingPoint(true);
        _dnode.setEnableUvMap(true);
        _dnode.setEnableAccelerometer(true);
        _dnode.setConfiguration(config);
    } catch (ArgumentException& e) {
        std::printf("DEPTH Argument Exception: %s\n", e.what());
    } catch (UnauthorizedAccessException& e) {
        std::printf("DEPTH Unauthorized Access Exception: %s\n", e.what());
    } catch (IOException& e) {
        std::printf("DEPTH IO Exception: %s\n", e.what());
    } catch (InvalidOperationException& e) {
        std::printf("DEPTH Invalid Operation Exception: %s\n", e.what());
    } catch (ConfigurationException& e) {
        std::printf("DEPTH Configuration Exception: %s\n", e.what());
    } catch (StreamingException& e) {
        std::printf("DEPTH Streaming Exception: %s\n", e.what());
    } catch (TimeoutException&) {
        std::printf("DEPTH Timeout Exception\n");
    }
}

void DS325::configureColorNode(Node node) {
    _cnode = node.as<ColorNode>();

    ColorNode::Configuration config = _cnode.getConfiguration();
    config.frameFormat = _format;
    config.compression = _compression;
    config.framerate = 30;
    config.powerLineFrequency = POWER_LINE_FREQUENCY_50HZ;

    try {
        _context.requestControl(_cnode, 0);
        _cnode.newSampleReceivedEvent().connect(this, &DS325::onNewColorSample);
        _cnode.setEnableColorMap(true);
//        _color.setBrightness(0);
//        _color.setContrast(5);
//        _color.setSaturation(5);
//        _color.setHue(0);
//        _color.setGamma(3);
//        _color.setWhiteBalance(4650);
//        _color.setSharpness(5);
        _cnode.setWhiteBalanceAuto(true);
        _cnode.setConfiguration(config);
    } catch (ArgumentException& e) {
        std::printf("COLOR Argument Exception: %s\n", e.what());
    } catch (UnauthorizedAccessException& e) {
        std::printf("COLOR Unauthorized Access Exception: %s\n", e.what());
    } catch (IOException& e) {
        std::printf("COLOR IO Exception: %s\n", e.what());
    } catch (InvalidOperationException& e) {
        std::printf("COLOR Invalid Operation Exception: %s\n", e.what());
    } catch (ConfigurationException& e) {
        std::printf("COLOR Configuration Exception: %s\n", e.what());
    } catch (StreamingException& e) {
        std::printf("COLOR Streaming Exception: %s\n", e.what());
    } catch (TimeoutException&) {
        std::printf("COLOR Timeout Exception\n");
    }
}

void DS325::configureAudioNode(Node node) {
    _anode = node.as<AudioNode>();

    AudioNode::Configuration config = _anode.getConfiguration();
    config.sampleRate = 44100;

    try {
        _context.requestControl(_anode, 0);
        _anode.newSampleReceivedEvent().connect(this, &DS325::onNewAudioSample);
        _anode.setConfiguration(config);
        _anode.setInputMixerLevel(0.5f);
    } catch (ArgumentException& e) {
        std::printf("AUDIO Argument Exception: %s\n", e.what());
    } catch (UnauthorizedAccessException& e) {
        std::printf("AUDIO Unauthorized Access Exception: %s\n", e.what());
    } catch (ConfigurationException& e) {
        std::printf("AUDIO Configuration Exception: %s\n", e.what());
    } catch (StreamingException& e) {
        std::printf("AUDIO Streaming Exception: %s\n", e.what());
    } catch (TimeoutException&) {
        std::printf("AUDIO Timeout Exception\n");
    }
}

}
