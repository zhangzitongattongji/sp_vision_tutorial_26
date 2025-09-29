#include "io/my_camera.hpp"
#include "tasks/yolo.hpp"
#include "opencv2/opencv.hpp"
#include "tools/img_tools.hpp"
#include<iostream>
using namespace std;

int main()
{
    // 初始化相机、yolo类
    myCamera camera;

    string yolopath="./configs/yolo.yaml";
    auto_aim::YOLO armorDetector(yolopath);
    while (1) {
        // 调用相机读取图像
        cv::Mat frame = camera.read();

        if (frame.empty()) {
            std::cerr << "Failed to capture an image!" << std::endl;
            break;
        }

        // 调用yolo识别装甲板
       
        std::list<auto_aim::Armor> armors = armorDetector.detect(frame);
        
          for (const auto& armor : armors) {
            const auto& points = armor.points; // 获取装甲板的关键点
            if (points.size() == 4) { // 确保有四个关键节点
                // 将四个点连接成一个矩形
                cv::line(frame, points[0], points[1], cv::Scalar(0, 0, 255), 2); // 红色线条
                cv::line(frame, points[1], points[2], cv::Scalar(0, 0, 255), 2);
                cv::line(frame, points[2], points[3], cv::Scalar(0, 0, 255), 2);
                cv::line(frame, points[3], points[0], cv::Scalar(0, 0, 255), 2);
            }
        }
            // 4. 显示结果图像
            cv::resize(frame, frame, cv::Size(640, 480)); // 调整图像大小以便显示
            cv::imshow("image", frame);

            // 按下 'q' 键退出循环
            if (cv::waitKey(1) == 'q') {
                break;
            }


        // 显示图像
        // cv::resize(img, img , cv::Size(640, 480));
        // cv::imshow("img", img);
        // if (cv::waitKey(0) == 'q') {
        //     // break;
        // }
    }

    return 0;
}