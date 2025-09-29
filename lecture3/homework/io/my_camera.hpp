#ifndef MYCAMERA_HPP
#define MYCAMERA_HPP

#include "hikrobot/include/MvCameraControl.h"
#include <opencv2/opencv.hpp>

class myCamera {
public:
    // 构造函数：初始化相机
    myCamera();
    // 析构函数：释放资源
    ~myCamera();

    // 读取一帧图像
    cv::Mat read();

private:
    void* handle_;                  // 相机句柄
    MV_CC_DEVICE_INFO_LIST device_list_; // 设备列表
    bool is_opened_;                // 相机是否已打开
    cv::Mat transfer(MV_FRAME_OUT& raw);
};

#endif