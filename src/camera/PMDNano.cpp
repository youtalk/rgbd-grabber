/**
 * @file PMDNano.cpp
 * @author Yutaka Kondo <yutaka.kondo@youtalk.jp>
 * @date Jul 9, 2013
 */
#include "rgbd/camera/PMDNano.h"

namespace rgbd {

PMDNano::PMDNano(const std::string& srcPlugin, const std::string& procPlugin,
                 const std::string& srcParam, const std::string& procParam) :
        DepthCamera(),
        _running(false) {
    open(srcPlugin, procPlugin, srcParam, procParam);

    std::cout << "PMDNano: opened" << std::endl;
}

PMDNano::~PMDNano() {
    boost::mutex::scoped_lock lock(_mutex);

    _running = false;
    delete[] _source;
    delete[] _buffer;
    delete[] _vbuffer;
    pmdClose(_handle);

    std::cout << "PMDNano: closed" << std::endl;
}

cv::Size PMDNano::depthSize() const {
    return cv::Size(_width, _height);
}

void PMDNano::start() {
    _running = true;
    boost::thread thread(boost::bind(&PMDNano::update, this));

    if (pmdGetSourceDataDescription(_handle, &_description) != PMD_OK)
        closeByError("pmdGetSourceDataDescription");
    if (_description.subHeaderType != PMD_IMAGE_DATA) {
        std::cerr << "source is not an image." << std::endl;
        pmdClose(_handle);
        std::exit(-1);
    }

    _width = _description.img.numColumns;
    _height = _description.img.numRows;
    _size = _width * _height;
    _source = new char[_description.size];
    _buffer = new float[_size];
    _vbuffer = new float[3 * _size];

    if (pmdGetSourceData(_handle, _source, _description.size) != PMD_OK)
        closeByError("pmdGetSourceData");
}

void PMDNano::update() {
    while (_running) {
        {
            boost::mutex::scoped_lock lock(_mutex);

            if (pmdUpdate(_handle) != PMD_OK)
                closeByError("pmdUpdate");
        }
        usleep(11111); // 90[Hz]
    }
}

void PMDNano::captureDepth(cv::Mat& buffer) {
    boost::mutex::scoped_lock lock(_mutex);

    if (pmdGetDistances(_handle, _buffer, _size * sizeof (float)))
        closeByError("pmdGetDistances");

    std::memcpy(buffer.data, _buffer, _size * sizeof (float));
}

void PMDNano::captureAmplitude(cv::Mat& buffer) {
    boost::mutex::scoped_lock lock(_mutex);

    if (pmdGetAmplitudes(_handle, _buffer, _size * sizeof (float)))
        closeByError("pmdGetAmplitudes");

    std::memcpy(buffer.data, _buffer, _size * sizeof (float));
}

void PMDNano::capturePointCloud(PointCloud::Ptr buffer) {
    boost::mutex::scoped_lock lock(_mutex);
    size_t index = 0;

    if (pmdGet3DCoordinates(_handle, _vbuffer, 3 * _size * sizeof (float)))
        closeByError("pmdGet3DCoordinates");

    for (auto& point: buffer->points) {
        point.x = _vbuffer[3 * index];
        point.y = _vbuffer[3 * index + 1];
        point.z = _vbuffer[3 * index + 2];
        index++;
    }
}

void PMDNano::open(const std::string& srcPlugin, const std::string& procPlugin,
                   const std::string& srcParam, const std::string& procParam) {
    if (pmdOpen(&_handle, srcPlugin.c_str(), srcParam.c_str(),
                procPlugin.c_str(), procParam.c_str()) != PMD_OK)
        closeByError("pmdOpenSourcePlugin");
}

void PMDNano::closeByError(const std::string& function) {
    char error[128];

    pmdGetLastError(_handle, error, 128);
    std::cerr << function << ": " << error << std::endl;
    pmdClose(_handle);
    exit(1);
}

}
