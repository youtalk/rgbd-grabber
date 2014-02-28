/**
 * @file DS325.cpp
 * @author Yutaka Kondo <yutaka.kondo@youtalk.jp>
 * @date Jul 29, 2013
 */

#include "DS325.h"

namespace rgbd {

DS325::DS325(const size_t deviceNo, const DepthSense::FrameFormat frameFormat) :
        DepthCamera(),
        frameFormat_(frameFormat),
        depthSize_(320, 240),
        colorSize_(frameFormat == FRAME_FORMAT_VGA ? 640 : 1280,
                   frameFormat == FRAME_FORMAT_VGA ? 480 : 720),
        context_(Context::create("localhost")) {
    context_.deviceAddedEvent().connect(this, &DS325::onDeviceConnected);
    context_.deviceRemovedEvent().connect(this, &DS325::onDeviceDisconnected);
    std::vector<Device> devices = context_.getDevices();

    if (deviceNo < devices.size()) {
        devices[deviceNo].nodeAddedEvent().connect(this, &DS325::onNodeConnected);
        devices[deviceNo].nodeRemovedEvent().connect(this, &DS325::onNodeDisconnected);

        for (Node node: devices[deviceNo].getNodes()) {
            if (node.is<DepthNode>() && !depth_.isSet())
                configureDepthNode(node);
            else if (node.is<ColorNode>() && !color_.isSet())
                configureColorNode(node);
            else if (node.is<AudioNode>() && !audio_.isSet())
                configureAudioNode(node);

            context_.registerNode(node);
        }

        std::cout << "DS325: opened" << std::endl;
    } else {
        std::cerr << "DS325: camera " << deviceNo
                  << " cannot open" << std::endl;
        std::exit(-1);
    }

}

DS325::~DS325() {
    if (depth_.isSet())
        context_.unregisterNode(depth_);
    if (color_.isSet())
        context_.unregisterNode(color_);
    if (audio_.isSet())
        context_.unregisterNode(audio_);

    std::cout << "DS325: closed" << std::endl;
}

cv::Size DS325::depthSize() const {
    return depthSize_;
}

cv::Size DS325::colorSize() const {
    return colorSize_;
}

void DS325::update() {
    context_.startNodes();
    context_.run();
    context_.stopNodes();
}

void DS325::start() {
    boost::thread t(boost::bind(&DS325::update, this));
    sleep(3); // I'm not sure but it must be necessary
}

void DS325::captureDepth(cv::Mat& buffer) {
    boost::mutex::scoped_lock lock(depthMutex_);
    std::memcpy(buffer.data, ddata_.depthMap, ddata_.depthMap.size() * 2);
}

void DS325::captureAmplitude(cv::Mat& buffer) {
    boost::mutex::scoped_lock lock(depthMutex_);
    std::memcpy(buffer.data, ddata_.confidenceMap, ddata_.confidenceMap.size() * 2);
}

void DS325::captureColor(cv::Mat& buffer) {
    boost::mutex::scoped_lock lock(colorMutex_);
    std::memcpy(buffer.data, cdata_.colorMap, cdata_.colorMap.size());
}

void DS325::captureVertex(PointXYZRGBVector& buffer) {
    boost::mutex::scoped_lock lock(depthMutex_);
    std::size_t index = 0;

    for (auto& b: buffer) {
        auto& f = ddata_.verticesFloatingPoint[index++];
        b.x = f.x;
        b.y = f.y;
        b.z = f.z;
    }
}

void DS325::captureAudio(std::vector<uchar>& buffer) {
    boost::mutex::scoped_lock lock(audioMutex_);
    buffer.clear();

    for (std::size_t i = 0; i < adata_.audioData.size(); i++)
        buffer.push_back(adata_.audioData[i]);
}

void DS325::captureAcceleration(cv::Point3f& buffer) {
    boost::mutex::scoped_lock lock(depthMutex_);

    buffer.x = ddata_.acceleration.x;
    buffer.y = ddata_.acceleration.y;
    buffer.z = ddata_.acceleration.z;
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
        boost::mutex::scoped_lock lock(depthMutex_);
        ddata_ = data;
    }
}

void DS325::onNewColorSample(ColorNode node, ColorNode::NewSampleReceivedData data) {
    int width, height;
    FrameFormat_toResolution(data.captureConfiguration.frameFormat, &width, &height);

    {
        boost::mutex::scoped_lock lock(colorMutex_);
        cdata_ = data;
    }
}

void DS325::onNewAudioSample(AudioNode node, AudioNode::NewSampleReceivedData data) {
    boost::mutex::scoped_lock lock(audioMutex_);
    adata_ = data;
}

void DS325::configureDepthNode(Node node) {
    depth_ = node.as<DepthNode>();

    DepthNode::Configuration config = depth_.getConfiguration();
    config.frameFormat = FRAME_FORMAT_QVGA;
    config.framerate = 30;
    config.mode = DepthNode::CAMERA_MODE_CLOSE_MODE;
    config.saturation = false;

    try {
        context_.requestControl(depth_, 0);
        depth_.newSampleReceivedEvent().connect(this, &DS325::onNewDepthSample);
        depth_.setEnableDepthMap(true);
        depth_.setEnableConfidenceMap(true);
        depth_.setEnableVerticesFloatingPoint(true);
        depth_.setEnableAccelerometer(true);
        depth_.setConfiguration(config);
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
        std::printf("DEPTH TimeoutException\n");
    }
}

void DS325::configureColorNode(Node node) {
    color_ = node.as<ColorNode>();

    ColorNode::Configuration config = color_.getConfiguration();
    config.frameFormat = frameFormat_;
    config.compression = COMPRESSION_TYPE_MJPEG;
    config.framerate = 30;
    config.powerLineFrequency = POWER_LINE_FREQUENCY_50HZ;

    try {
        context_.requestControl(color_, 0);
        color_.newSampleReceivedEvent().connect(this, &DS325::onNewColorSample);
        color_.setEnableColorMap(true);
//         color_.setBrightness(0);
//         color_.setContrast(5);
//         color_.setSaturation(5);
//         color_.setHue(0);
//         color_.setGamma(3);
//         color_.setSharpness(5);
//         color_.setWhiteBalance(4650);
        color_.setWhiteBalanceAuto(true);
        color_.setConfiguration(config);
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
        std::printf("COLOR TimeoutException\n");
    }
}

void DS325::configureAudioNode(Node node) {
    audio_ = node.as<AudioNode>();

    AudioNode::Configuration config = audio_.getConfiguration();
    config.sampleRate = 44100;

    try {
        context_.requestControl(audio_, 0);
        audio_.newSampleReceivedEvent().connect(this, &DS325::onNewAudioSample);
        audio_.setConfiguration(config);
        audio_.setInputMixerLevel(0.5f);
    } catch (ArgumentException& e) {
        std::printf("Argument Exception: %s\n", e.what());
    } catch (UnauthorizedAccessException& e) {
        std::printf("Unauthorized Access Exception: %s\n", e.what());
    } catch (ConfigurationException& e) {
        std::printf("Configuration Exception: %s\n", e.what());
    } catch (StreamingException& e) {
        std::printf("Streaming Exception: %s\n", e.what());
    } catch (TimeoutException&) {
        std::printf("TimeoutException\n");
    }
}

}
