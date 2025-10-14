#ifndef AUTO_BUFF__SOLVER_HPP
#define AUTO_BUFF__SOLVER_HPP

#include <opencv2/opencv.hpp>
#include <vector>

namespace auto_buff
{

class Buff_Solver
{
public:
    
    Buff_Solver();
    
    
    bool solvePnP(const cv::Mat& camera_matrix,
              const cv::Mat& dist_coeffs,
              const std::vector<cv::Point3f>& object_points,
              const std::vector<cv::Point2f>& image_points,
              cv::Mat& rvec, cv::Mat& tvec);
    
   
    cv::Vec3f convert(const cv::Mat& rvec_input);
    
    
    cv::Mat camera_matrix;             
    cv::Mat dist_coeffs;                
    std::vector<cv::Point3f> object_points;  
    std::vector<cv::Point2f> image_points;   
    cv::Mat rvec;                       
    cv::Mat tvec;                       
    bool success;                       

private:
    
    bool is_initialized_;
};
}  
#endif  