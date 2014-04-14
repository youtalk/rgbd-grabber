/**
 * @file UEye.cpp
 * @author Yutaka Kondo <yutaka.kondo@kawadarobot.co.jp>
 * @date Apr 14, 2014
 */

#include "UEye.h"

namespace rgbd {

UEye::UEye(const uint32_t& deviceNo, const cv::Size& size, double fps) :
        _size(size), _camera(deviceNo), _fps(fps) {
    CAMINFO cinfo;
    uint32_t check;
    if (is_InitCamera(&_camera, &cinfo) != IS_SUCCESS) {
        std::cerr << "UEye: cannot opened" << std::endl;
        std::exit(-1);
    }

    SENSORINFO sinfo;
    int m_nSizeX = 0, m_nSizeY = 0;
    int m_MaxWidth = 0, m_MaxHeight = 0;
    int m_nMemoryId;
    is_GetCameraInfo(_camera, &cinfo);
    is_GetSensorInfo(_camera, &sinfo);
    if (m_nSizeX <= 0)
        m_nSizeX = m_MaxWidth = sinfo.nMaxWidth;
    if (m_nSizeY <= 0)
        m_nSizeY = m_MaxHeight = sinfo.nMaxHeight;
    std::cerr << "maxWidth: " << m_MaxWidth << std::endl;
    std::cerr << "maxHeight: " << m_MaxHeight << std::endl;
    if (_buffer != NULL)
        is_FreeImageMem(_camera, _buffer, m_nMemoryId);
    _buffer = NULL;

    int m_nColorMode = IS_CM_BGR8_PACKED;
    int m_nBitsPerPixel = 24;
    check = is_SetColorMode(_camera, m_nColorMode);
    std::cout << "colormode:" << check << std::endl;
    check = is_SetSubSampling(
            _camera, IS_SUBSAMPLING_2X_VERTICAL | IS_SUBSAMPLING_2X_HORIZONTAL);
    std::cout << "sub:" << check << std::endl;
    int aoi_x = 0, aoi_y = 0, aoi_w = _size.width, aoi_h = _size.height;

    IS_RECT rectAOI;
    rectAOI.s32X = aoi_x;
    rectAOI.s32Y = aoi_y;
    rectAOI.s32Width = aoi_w;
    rectAOI.s32Height = aoi_h;
    check = is_AOI(_camera, IS_AOI_IMAGE_SET_AOI, (void *) &rectAOI,
                  sizeof(rectAOI));

    std::cout << "aoi:" << check << std::endl;
    check = is_SetFrameRate(_camera, _fps, &_fps);
    std::cout << "fps:" << check << ", new_fps=" << _fps << std::endl;
    double new_exp = 0.0;

    double p1 = 1.0, p2 = 0.0;
    check = is_SetAutoParameter(_camera, IS_SET_ENABLE_AUTO_SHUTTER, &p1, &p2);
    check = is_Exposure(_camera, IS_EXPOSURE_CMD_GET_EXPOSURE, &new_exp,
                       sizeof(new_exp));

    std::cout << "exposure:" << check << ", new_exp=" << new_exp << std::endl;
    m_nSizeX = aoi_w;
    m_nSizeY = aoi_h;
    UINT nPixelClock = 30 * 2;
    check = is_PixelClock(_camera, IS_PIXELCLOCK_CMD_SET, (void*) &nPixelClock,
                         sizeof(nPixelClock));

    std::cout << "pixel:" << check << std::endl;

    check = is_AllocImageMem(_camera, m_nSizeX, m_nSizeY, m_nBitsPerPixel,
                            &_buffer, &m_nMemoryId);
    if (_buffer != NULL)
        check = is_SetImageMem(_camera, _buffer, m_nMemoryId);
    else
        std::exit(-1);

    if (sinfo.nColorMode == IS_COLORMODE_BAYER) {
        double dEnable = 1.0;
        check = is_SetAutoParameter(_camera, IS_SET_AUTO_WB_ONCE, &dEnable, 0);
    }

    _size = cv::Size(m_nSizeX, m_nSizeY);
}

UEye::~UEye() {
}

cv::Size UEye::colorSize() const {
    return _size;
}

void UEye::start() {
    is_CaptureVideo(_camera, IS_DONT_WAIT);
    boost::thread t(boost::bind(&UEye::update, this));
}

void UEye::update() {
}

void UEye::captureColor(cv::Mat& buffer) {
    std::memcpy(buffer.data, (uchar*)_buffer, 3 * sizeof(uchar) * buffer.rows * buffer.cols);
}

}
