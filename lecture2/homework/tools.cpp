#include "tools.hpp"
#include <opencv2/opencv.hpp>


// 缩放函数实现 - C++11兼容版本
ResizeResult resize_to_canvas(const cv::Mat& input_image, int canvas_size) {
    // 获取输入图像的尺寸
    int input_width = input_image.cols;
    int input_height = input_image.rows;

    // 计算缩放比例
    double scale_width = static_cast<double>(canvas_size) / input_width;
    double scale_height = static_cast<double>(canvas_size) / input_height;
    double scale = std::min(scale_width, scale_height); // 等比例缩放

    // 计算缩放后的图像尺寸
    int resized_width = static_cast<int>(input_width * scale);
    int resized_height = static_cast<int>(input_height * scale);

    // 缩放图像
    cv::Mat resized_image;
    cv::resize(input_image, resized_image, cv::Size(resized_width, resized_height));

    // 创建黑色画布
    cv::Mat canvas(canvas_size, canvas_size, CV_8UC3, cv::Scalar(0, 0, 0));

    // 将缩放后的图像放置在画布中间
    int x_offset = (canvas_size - resized_width) / 2;
    int y_offset = (canvas_size - resized_height) / 2;
    cv::Rect roi(x_offset, y_offset, resized_width, resized_height);
    resized_image.copyTo(canvas(roi));

    // 返回结果 - C++11方式
    ResizeResult result;
    result.canvas = canvas;
    result.scale_width = scale_width;
    result.scale_height = scale_height;
    
    return result;
}
