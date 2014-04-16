/**
 * @file UEye.cpp
 * @author Yutaka Kondo <yutaka.kondo@kawadarobot.co.jp>
 * @date Apr 14, 2014
 */

#include "UEye.h"

namespace rgbd {

UEye::UEye(const uint deviceNo, const cv::Size& size, double fps) :
        _size(size), _camera(deviceNo) {
    CAMINFO cinfo;
    if (is_InitCamera(&_camera, &cinfo) != IS_SUCCESS) {
        std::cerr << "UEye: cannot opened" << std::endl;
        std::exit(-1);
    }

    SENSORINFO sinfo;
    is_GetCameraInfo(_camera, &cinfo);
    is_GetSensorInfo(_camera, &sinfo);

    std::cerr << "UEye: max width = " << sinfo.nMaxWidth << std::endl;
    std::cerr << "UEye: max height = " << sinfo.nMaxHeight << std::endl;

    int colorMode = IS_CM_BGR8_PACKED;
    int bitsPerPixel = 24;
    is_SetColorMode(_camera, colorMode);
    is_SetSubSampling(_camera, IS_SUBSAMPLING_2X_VERTICAL | IS_SUBSAMPLING_2X_HORIZONTAL);

    IS_RECT rect;
    rect.s32X = 0;
    rect.s32Y = 0;
    rect.s32Width = _size.width;
    rect.s32Height = _size.height;
    is_AOI(_camera, IS_AOI_IMAGE_SET_AOI, (void*)&rect, sizeof(rect));

    is_SetFrameRate(_camera, fps, &fps);
    std::cout << "UEye: fps = " << fps << std::endl;

    double exposure = 0.0;
    double p1 = 1.0, p2 = 0.0;
    is_SetAutoParameter(_camera, IS_SET_ENABLE_AUTO_SHUTTER, &p1, &p2);
    is_Exposure(_camera, IS_EXPOSURE_CMD_GET_EXPOSURE, &exposure, sizeof(exposure));
    std::cout << "UEye: exposure = " << exposure << std::endl;

    uint pixelClock = 30 * 2;
    is_PixelClock(_camera, IS_PIXELCLOCK_CMD_SET, &pixelClock, sizeof(pixelClock));
    is_AllocImageMem(_camera, _size.width, _size.height, bitsPerPixel,
                     &_buffer, &_bufferId);
    if (_buffer != NULL)
        is_SetImageMem(_camera, _buffer, _bufferId);
    else
        std::exit(-1);

    if (sinfo.nColorMode == IS_COLORMODE_BAYER) {
        double enable = 1.0;
        double disable = 0.0;
        is_SetAutoParameter(_camera, IS_SET_WB_AUTO_ENABLE_ONCE, &disable, 0);
    }
}

UEye::UEye(const uint deviceNo, const std::string& file) :
        _camera(deviceNo), _size(752, 480) {
    CAMINFO cinfo;
    if (is_InitCamera(&_camera, &cinfo) != IS_SUCCESS) {
        std::cerr << "UEye: cannot open" << std::endl;
        std::exit(-1);
    }

    const std::wstring wfile(file.begin(), file.end());
    if (is_ParameterSet(_camera, IS_PARAMETERSET_CMD_LOAD_FILE,
                        (void*)file.c_str(), 0) != IS_SUCCESS) {
        std::cerr << "UEye: cannot load " << file << std::endl;
        std::exit(-1);
    }
}

UEye::~UEye() {
    is_FreeImageMem(_camera, _buffer, _bufferId);
}

cv::Size UEye::colorSize() const {
    return _size;
}

void UEye::start() {
    is_CaptureVideo(_camera, IS_DONT_WAIT);
}

void UEye::captureColor(cv::Mat& buffer) {
    std::memcpy(buffer.data, _buffer,
                3 * sizeof(uchar) * buffer.rows * buffer.cols);
}

}
