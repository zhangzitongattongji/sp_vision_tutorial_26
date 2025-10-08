#include "my_camera.hpp"

// 构造函数：初始化相机
myCamera::myCamera() : handle_(nullptr), is_opened_(false) {
    int ret = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list_);
    if (ret != MV_OK || device_list_.nDeviceNum == 0) {
        throw std::runtime_error("No camera device found!");
    }

    ret = MV_CC_CreateHandle(&handle_, device_list_.pDeviceInfo[0]);
    if (ret != MV_OK) {
        throw std::runtime_error("Failed to create camera handle!");
    }

    ret = MV_CC_OpenDevice(handle_);
    if (ret != MV_OK) {
        MV_CC_DestroyHandle(handle_);
        throw std::runtime_error("Failed to open camera!");
    }

    MV_CC_SetEnumValue(handle_, "BalanceWhiteAuto", MV_BALANCEWHITE_AUTO_CONTINUOUS);
    MV_CC_SetEnumValue(handle_, "ExposureAuto", MV_EXPOSURE_AUTO_MODE_OFF);
    MV_CC_SetEnumValue(handle_, "GainAuto", MV_GAIN_MODE_OFF);
    MV_CC_SetFloatValue(handle_, "ExposureTime", 2000);
    MV_CC_SetFloatValue(handle_, "Gain", 20);
    MV_CC_SetFrameRate(handle_, 60);

    is_opened_ = true;
}

// 析构函数：释放资源
myCamera::~myCamera() {
    if (is_opened_) {
        MV_CC_StopGrabbing(handle_);
        MV_CC_CloseDevice(handle_);
    }
    if (handle_) {
        MV_CC_DestroyHandle(handle_);
    }
}

cv::Mat myCamera::transfer(MV_FRAME_OUT& raw) {
    MV_CC_PIXEL_CONVERT_PARAM cvt_param;
    cv::Mat img(cv::Size(raw.stFrameInfo.nWidth, raw.stFrameInfo.nHeight), CV_8U, raw.pBufAddr);

    cvt_param.nWidth = raw.stFrameInfo.nWidth;
    cvt_param.nHeight = raw.stFrameInfo.nHeight;

    cvt_param.pSrcData = raw.pBufAddr;
    cvt_param.nSrcDataLen = raw.stFrameInfo.nFrameLen;
    cvt_param.enSrcPixelType = raw.stFrameInfo.enPixelType;

    cvt_param.pDstBuffer = img.data;
    cvt_param.nDstBufferSize = img.total() * img.elemSize();
    cvt_param.enDstPixelType = PixelType_Gvsp_BGR8_Packed;

    auto pixel_type = raw.stFrameInfo.enPixelType;
    const static std::unordered_map<MvGvspPixelType, cv::ColorConversionCodes> type_map = {
        {PixelType_Gvsp_BayerGR8, cv::COLOR_BayerGR2RGB},
        {PixelType_Gvsp_BayerRG8, cv::COLOR_BayerRG2RGB},
        {PixelType_Gvsp_BayerGB8, cv::COLOR_BayerGB2RGB},
        {PixelType_Gvsp_BayerBG8, cv::COLOR_BayerBG2RGB}};
    cv::cvtColor(img, img, type_map.at(pixel_type));

    return img;
}
// 读取一帧图像
cv::Mat myCamera::read() {
    if (!is_opened_) {
        throw std::runtime_error("Camera is not opened!");
    }

    MV_FRAME_OUT raw;
    unsigned int nMsec = 100;

    int ret = MV_CC_StartGrabbing(handle_);
    if (ret != MV_OK) {
        throw std::runtime_error("Failed to start grabbing!");
    }

    ret = MV_CC_GetImageBuffer(handle_, &raw, nMsec);
    if (ret != MV_OK) {
        MV_CC_StopGrabbing(handle_);
        throw std::runtime_error("Failed to get image buffer!");
    }

    cv::Mat img = this->transfer(raw); // 调用原始转换函数
    
    ret = MV_CC_FreeImageBuffer(handle_, &raw);
  
    MV_CC_StopGrabbing(handle_);
    return img;
}

