#ifndef TOOLS_HPP
#define TOOLS_HPP

#include <opencv2/opencv.hpp>

// 定义结构体
struct ResizeResult {
    cv::Mat canvas;
    double scale_width;
    double scale_height;
};

// 函数声明
ResizeResult resize_to_canvas(const cv::Mat& input_image, int canvas_size);

#endif // TOOLS_HPP
