/**
 * @file PMDNano.cpp
 * @author Yutaka Kondo <yutaka.kondo@youtalk.jp>
 * @date Jul 9, 2013
 */
#include "PMDNano.h"

namespace rgbd {

PMDNano::PMDNano(const std::string& srcPlugin, const std::string& procPlugin,
                 const std::string& srcParam, const std::string& procParam) :
        DepthCamera(),
        running_(true) {
    open(srcPlugin, procPlugin, srcParam, procParam);

    std::cout << "PMDNano: opened" << std::endl;
}

PMDNano::~PMDNano() {
    boost::mutex::scoped_lock lock(mutex_);

    running_ = false;
    delete[] source_;
    delete[] buffer_;
    delete[] vertexBuffer_;
    pmdClose(handle_);

    std::cout << "PMDNano: closed" << std::endl;
}

cv::Size PMDNano::depthSize() const {
    return cv::Size(width_, height_);
}

void PMDNano::start() {
    boost::thread thread(boost::bind(&PMDNano::update, this));

    if (pmdGetSourceDataDescription(handle_, &description_) != PMD_OK)
        closeByError("pmdGetSourceDataDescription");
    if (description_.subHeaderType != PMD_IMAGE_DATA) {
        std::cerr << "source is not an image." << std::endl;
        pmdClose(handle_);
        std::exit(-1);
    }

    width_ = description_.img.numColumns;
    height_ = description_.img.numRows;
    size_ = width_ * height_;
    source_ = new char[description_.size];
    buffer_ = new float[size_];
    vertexBuffer_ = new float[3 * size_];

    if (pmdGetSourceData(handle_, source_, description_.size) != PMD_OK)
        closeByError("pmdGetSourceData");
}

bool PMDNano::running() {
    boost::mutex::scoped_lock lock(mutex_);

    return running_;
}

void PMDNano::update() {
    while (running()) {
        {
            boost::mutex::scoped_lock lock(mutex_);

            if (pmdUpdate(handle_) != PMD_OK)
                closeByError("pmdUpdate");
        }
        usleep(11111); // 90[Hz]
    }
}

void PMDNano::captureDepth(cv::Mat& buffer) {
    boost::mutex::scoped_lock lock(mutex_);

    if (pmdGetDistances(handle_, buffer_, size_ * sizeof (float)))
        closeByError("pmdGetDistances");

    std::memcpy(buffer.data, buffer_, size_ * sizeof (float));
}

void PMDNano::captureAmplitude(cv::Mat& buffer) {
    boost::mutex::scoped_lock lock(mutex_);

    if (pmdGetAmplitudes(handle_, buffer_, size_ * sizeof (float)))
        closeByError("pmdGetAmplitudes");

    std::memcpy(buffer.data, buffer_, size_ * sizeof (float));
}

void PMDNano::captureVertex(PointXYZRGBVector& buffer) {
    boost::mutex::scoped_lock lock(mutex_);

    if (pmdGet3DCoordinates(handle_, vertexBuffer_, 3 * size_ * sizeof (float)))
        closeByError("pmdGet3DCoordinates");

    buffer.resize(width_ * height_);

    for (size_t i = 0; i < width_ * height_; i++) {
        buffer[i].x = vertexBuffer_[3 * i];
        buffer[i].y = vertexBuffer_[3 * i + 1];
        buffer[i].z = vertexBuffer_[3 * i + 2];
    }
}

void PMDNano::open(const std::string& srcPlugin, const std::string& procPlugin,
                   const std::string& srcParam, const std::string& procParam) {
    if (pmdOpen(&handle_, srcPlugin.c_str(), srcParam.c_str(),
                procPlugin.c_str(), procParam.c_str()) != PMD_OK)
        closeByError("pmdOpenSourcePlugin");
}

void PMDNano::closeByError(const std::string& function) {
    char error[128];

    pmdGetLastError(handle_, error, 128);
    std::cerr << function << ": " << error << std::endl;
    pmdClose(handle_);
    exit(1);
}

}
