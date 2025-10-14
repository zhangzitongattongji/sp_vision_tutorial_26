#include "buff_solver.hpp"
#include <iostream>
#include <cmath>

namespace auto_buff
{

Buff_Solver::Buff_Solver() : is_initialized_(true), success(false) {
    
    rvec = cv::Mat::zeros(3, 1, CV_64F);
    tvec = cv::Mat::zeros(3, 1, CV_64F);
}


bool Buff_Solver::solvePnP(const cv::Mat& camera_matrix,
              const cv::Mat& dist_coeffs,
              const std::vector<cv::Point3f>& object_points,
              const std::vector<cv::Point2f>& image_points,
              cv::Mat& rvec, cv::Mat& tvec) {
    success = false;
    
   
    if (camera_matrix.empty() || dist_coeffs.empty()) {
        std::cerr << "Error: Camera parameters not provided!" << std::endl;
        return success;
    }
    
   
    if (object_points.size() != image_points.size()) {
        std::cerr << "Error: Object points and image points size mismatch!" << std::endl;
        return success;
    }
    
    
    if (object_points.empty()) {
        std::cerr << "Error: No object points!" << std::endl;
        return success;
    }
    
    
    success = cv::solvePnP(object_points, image_points, camera_matrix, 
                          dist_coeffs, rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);
    return success;
    }

cv::Vec3f Buff_Solver::convert(const cv::Mat& rvec_input) {
    
    cv::Vec3f euler_angles;
    
    
    cv::Mat rotation_matrix;
    cv::Rodrigues(rvec_input, rotation_matrix);
    
    
    double sy = sqrt(rotation_matrix.at<double>(0,0) * rotation_matrix.at<double>(0,0) +
                    rotation_matrix.at<double>(1,0) * rotation_matrix.at<double>(1,0));
    
    
    bool singular = sy < 1e-6;
    
    
    if (!singular) {
       
        euler_angles[0] = atan2(rotation_matrix.at<double>(2,1), rotation_matrix.at<double>(2,2));  // roll
        euler_angles[1] = atan2(-rotation_matrix.at<double>(2,0), sy);                              // pitch
        euler_angles[2] = atan2(rotation_matrix.at<double>(1,0), rotation_matrix.at<double>(0,0));  // yaw
    } else {
       
        euler_angles[0] = atan2(-rotation_matrix.at<double>(1,2), rotation_matrix.at<double>(1,1)); // roll
        euler_angles[1] = atan2(-rotation_matrix.at<double>(2,0), sy);                              // pitch
        euler_angles[2] = 0;                                                                        // yaw
    }
    
    return euler_angles;
}

}  