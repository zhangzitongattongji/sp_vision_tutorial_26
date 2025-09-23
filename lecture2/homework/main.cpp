#include <fmt/core.h>
#include <opencv2/opencv.hpp>
#include "tools.hpp"

int main() {
    // 读取图片
    cv::Mat image = cv::imread("heihei.jpg");
    if (image.empty()) {
        fmt::print("Error: Unable to load image!\n");
        return -1;
    }

    int canvas_size = 640;
    

    ResizeResult result = resize_to_canvas(image, canvas_size);
    

    // 显示结果
    cv::imshow("转变后", result.canvas); // 使用结构体成员
    cv::waitKey(0);

    // 使用 fmt 输出缩放参数
    fmt::print("Resize Parameters:\n");
    fmt::print("  Scale Width: {:.3f}\n", result.scale_width);
    fmt::print("  Scale Height: {:.3f}\n", result.scale_height);

    return 0;
}
