// #include "tasks/buff_detector.hpp"
// #include "io/camera.hpp"
// #include <chrono>
// #include <opencv2/opencv.hpp>

// int main()
// {
//     io::Camera camera(2.5, 16.9, "2bdf:0001");
//     std::chrono::steady_clock::time_point timestamp;  
//     auto_buff::Buff_Detector detector;
//     while(true){
//         cv::Mat img;
//         camera.read(img, timestamp);
//         auto fanblades = detector.detect(img);
//     }
//     return 0;
// }
#include "tasks/buff_detector.hpp"
#include "tasks/buff_solver.hpp"
#include <chrono>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include "tools/plotter.hpp"
#include "io/camera.hpp"
#include <deque>

// 相机内参矩阵 - 描述相机的内部参数
static const cv::Mat camera_matrix =
    (cv::Mat_<double>(3, 3) << 1286.307063384126, 0, 645.34450819155256,
                               0, 1288.1400736562441, 483.6163720308021,
                               0, 0, 1);

// 相机畸变系数 - 描述镜头畸变
static const cv::Mat distort_coeffs =
    (cv::Mat_<double>(1, 5) << -0.47562935060124745, 0.21831745829617311, 
       0.0004957613589406044, -0.00034617769548693592, 0);

// 扇叶关键点尺寸参数（单位：米）
static const double FANBLADE_RADIUS = 0.1;
// 扇叶绕圆心转动的半径（单位：米）
static const double ROTATION_RADIUS = 0.3;

// 圆心计算相关变量
static std::deque<cv::Point3f> position_history;  // 存储历史位置
static cv::Point3f estimated_center(0, 0, 0);     // 估计的圆心位置
static bool center_initialized = false;           // 圆心是否已初始化
static const int HISTORY_SIZE = 30;               // 历史数据窗口大小  

// 扇叶6个关键点的3D坐标（在扇叶坐标系下）
static const std::vector<cv::Point3f> fanblade_object_points{
    {-FANBLADE_RADIUS, -FANBLADE_RADIUS, 0},  // 点 0: 左下角
    {FANBLADE_RADIUS, -FANBLADE_RADIUS, 0},   // 点 1: 右下角
    {FANBLADE_RADIUS, FANBLADE_RADIUS, 0},    // 点 2: 右上角
    {-FANBLADE_RADIUS, FANBLADE_RADIUS, 0},   // 点 3: 左上角
    {0, 0, 0},                               // 点 4: 中心点
    {0, FANBLADE_RADIUS, 0}                  // 点 5: 上边中点
};

// 更新圆心估计的函数
void updateCenterEstimate(const cv::Point3f& current_position) {
    position_history.push_back(current_position);
    
    // 保持历史数据窗口大小
    if (position_history.size() > HISTORY_SIZE) {
        position_history.pop_front();
    }
    
    // 如果历史数据足够，计算圆心
    if (position_history.size() >= 10) {
        // 使用最小二乘法拟合圆心
        // 计算位置的平均值作为初始圆心估计
        cv::Point3f mean_pos(0, 0, 0);
        for (const auto& pos : position_history) {
            mean_pos.x += pos.x;
            mean_pos.y += pos.y;
            mean_pos.z += pos.z;
        }
        mean_pos.x /= position_history.size();
        mean_pos.y /= position_history.size();
        mean_pos.z /= position_history.size();
        
        // 基于已知半径和历史位置数据优化圆心估计
        cv::Point3f sum_center(0, 0, 0);
        int valid_estimates = 0;
        
        for (const auto& pos : position_history) {
            // 计算从mean_pos到当前位置的向量
            cv::Point3f vec = pos - mean_pos;
            float distance = sqrt(vec.x*vec.x + vec.y*vec.y + vec.z*vec.z);
            
            if (distance > 0.01) { // 避免除零
                // 将向量归一化并乘以已知半径
                vec = vec * (ROTATION_RADIUS / distance);
                // 圆心 = 当前位置 - 半径向量
                cv::Point3f center_estimate = pos - vec;
                sum_center += center_estimate;
                valid_estimates++;
            }
        }
        
        if (valid_estimates > 0) {
            estimated_center = sum_center * (1.0f / valid_estimates);
            center_initialized = true;
        }
    }
}

int main()
{
   
    // io::Camera camera(2.5, 16.9, "2bdf:0001");  
    // std::chrono::steady_clock::time_point timestamp;  
    
    // auto_buff::Buff_Detector detector;  
    // auto_buff::Buff_Solver solver;      
    // tools::Plotter plotter;             
    
    
    // while(true){
    //     cv::Mat img;           // 当前帧图像
    //     // 从摄像头读取图像
    //     camera.read(img, timestamp);  // 使用camera.read替代cap >>
        
    //     // 检查图像是否成功读取
    //     if (img.empty()) {
    //         std::cout << "无法从摄像头读取帧" << std::endl;
    //         continue;  // 继续尝试读取下一帧
    //     }
        
    //     // 使用检测器检测扇叶
    //     auto fanblades = detector.detect(img);
    // 打开视频文件
    cv::VideoCapture cap("assets/test.avi"); 
    
    // 检查视频文件是否成功打开
    if (!cap.isOpened()) {
        std::cerr << "无法打开视频文件!" << std::endl;
        return -1;
    }
    
    // 初始化检测器和求解器
    auto_buff::Buff_Detector detector;  // 扇叶检测器
    auto_buff::Buff_Solver solver;      // 位姿求解器
    tools::Plotter plotter;             // 数据可视化工具
    
    // 设置求解器的相机参数和物体点（一次性设置）
    solver.camera_matrix = camera_matrix;
    solver.dist_coeffs = distort_coeffs;
    solver.object_points = fanblade_object_points;
    
    // 主循环：逐帧处理视频
    while(true){
        cv::Mat img;           // 当前帧图像
        cap >> img;            // 读取下一帧
        
        // 检查是否到达视频末尾
        if (img.empty()) {
            std::cout << "视频播放完毕或无法读取帧" << std::endl;
            break;
        }
        
        // 使用检测器检测扇叶
        auto fanblades = detector.detect(img);
        cv::Mat display_img = img.clone();
        
        // 处理每个检测到的扇叶
        for (const auto& fanblade : fanblades) {
            // 根据扇叶类型设置颜色和标签
            cv::Scalar color;
            std::string type_name;
            switch (fanblade.type) {
                case auto_buff::_target:
                    color = cv::Scalar(0, 255, 0);   // 绿色
                    type_name = "_target";
                    break;
                case auto_buff::_light:
                    color = cv::Scalar(0, 255, 255); // 黄色
                    type_name = "_light";
                    break;
                case auto_buff::_unlight:
                    color = cv::Scalar(0, 0, 255);   // 红色
                    type_name = "_unlight";
                    break;
            }
            
            // 绘制关键点
            for (size_t i = 0; i < fanblade.points.size(); ++i) {
                // 绘制关键点小圆圈
                cv::circle(display_img, fanblade.points[i], 3, color, -1);
                // 在关键点旁边标注序号
                cv::putText(display_img, std::to_string(i), 
                           cv::Point(fanblade.points[i].x + 5, fanblade.points[i].y - 5),
                           cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1);
            }
            
            // 绘制中心点
            cv::circle(display_img, fanblade.center, 5, color, -1);
            cv::putText(display_img, "CENTER", 
                       cv::Point(fanblade.center.x + 10, fanblade.center.y - 10),
                       cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1);
            
            // 绘制类型标签
            cv::putText(display_img, type_name, 
                       cv::Point(fanblade.center.x - 20, fanblade.center.y - 20),
                       cv::FONT_HERSHEY_SIMPLEX, 0.7, color, 2);
            
            // PnP位姿求解
            if (fanblade.points.size() >= 6) {
                // 提取图像点用于PnP求解（使用所有6个关键点）
                std::vector<cv::Point2f> image_points;
                for (const auto& point : fanblade.points) {
                    // 转换为float类型并添加到向量
                    image_points.push_back(cv::Point2f(static_cast<float>(point.x), 
                                                      static_cast<float>(point.y)));
                }
                
                // 声明旋转向量和平移向量
                cv::Mat rvec, tvec;
                // 调用求解器进行PnP计算
                bool success = solver.solvePnP(camera_matrix, distort_coeffs, 
                                              fanblade_object_points, image_points, 
                                              rvec, tvec);
                
                // 如果求解成功
                if (success) {
                    // 计算旋转矩阵（从旋转向量转换）
                    cv::Mat rotation_matrix;
                    cv::Rodrigues(rvec, rotation_matrix);
                    
                    // 转换为欧拉角
                    cv::Vec3f euler_angles = solver.convert(rotation_matrix);
                    
                    // 更新圆心估计
                    cv::Point3f current_position(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));
                    updateCenterEstimate(current_position);
                    
                    // 获取估计的圆心坐标
                    double center_x = estimated_center.x;
                    double center_y = estimated_center.y;
                    double center_z = estimated_center.z;
                    
                    // 在图像上显示位置信息
                    std::string position_text = "Pos: (" + 
                        std::to_string(tvec.at<double>(0)).substr(0, 5) + ", " +
                        std::to_string(tvec.at<double>(1)).substr(0, 5) + ", " +
                        std::to_string(tvec.at<double>(2)).substr(0, 5) + ")";
                    
                    cv::putText(display_img, position_text,
                               cv::Point(fanblade.center.x - 50, fanblade.center.y + 40),
                               cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
                    
                    // 显示欧拉角
                    std::string euler_text = "Euler: (" + 
                        std::to_string(euler_angles[0]).substr(0, 5) + ", " +
                        std::to_string(euler_angles[1]).substr(0, 5) + ", " +
                        std::to_string(euler_angles[2]).substr(0, 5) + ")";
                    
                    cv::putText(display_img, euler_text,
                               cv::Point(fanblade.center.x - 50, fanblade.center.y + 60),
                               cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
                    
                    // 显示圆心坐标
                    std::string center_text;
                    if (center_initialized) {
                        center_text = "Center: (" + 
                            std::to_string(center_x).substr(0, 5) + ", " +
                            std::to_string(center_y).substr(0, 5) + ", " +
                            std::to_string(center_z).substr(0, 5) + ")";
                    } else {
                        center_text = "Center: Initializing... (" + std::to_string(position_history.size()) + "/10)";
                    }
                    
                    cv::putText(display_img, center_text,
                               cv::Point(fanblade.center.x - 50, fanblade.center.y + 80),
                               cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1);
                    
                    // 准备发送到PlotJuggler的数据
                    nlohmann::json pnp_data;
                    pnp_data["position_x"] = tvec.at<double>(0);     // 扇叶X坐标
                    pnp_data["position_y"] = tvec.at<double>(1);     // 扇叶Y坐标
                    pnp_data["position_z"] = tvec.at<double>(2);     // 扇叶Z坐标
                    pnp_data["rotation_roll"] = euler_angles[0];     // Roll角
                    pnp_data["rotation_pitch"] = euler_angles[1];    // Pitch角
                    pnp_data["rotation_yaw"] = euler_angles[2];      // Yaw角
                    pnp_data["center_x"] = center_x;                 // 圆心X坐标
                    pnp_data["center_y"] = center_y;                 // 圆心Y坐标
                    pnp_data["center_z"] = center_z;                 // 圆心Z坐标
                    pnp_data["rotation_radius"] = ROTATION_RADIUS;   // 转动半径
                    
                    // 发送数据到PlotJuggler
                    plotter.plot(pnp_data);
                } else {
                    // 求解失败时显示提示信息
                    cv::putText(display_img, "PNP Failed", 
                               cv::Point(fanblade.center.x - 30, fanblade.center.y + 40),
                               cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1);
                }
            }
        }
        
        // 显示检测结果（缩小至80%以便更好地查看）
        cv::resize(display_img, display_img, {}, 0.8, 0.8);
        cv::imshow("PNP Detection Results", display_img);
        // 按ESC键退出
        if (cv::waitKey(30) == 27) {
            break;
        }
    }
    
  
    cv::destroyAllWindows();
    return 0;
}