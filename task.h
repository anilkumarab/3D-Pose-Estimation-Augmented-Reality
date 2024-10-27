// Arun Srinivasan V K (002839500), and Abhinav Anil (002889398)
// 3/20/2024
// Purpose - This is a header file

#include <iostream>
#include <opencv2/opencv.hpp>

#ifndef TASKS_H
#define TASKS_H

struct Result // this store the first result of the code - corner_sets from the target, value stores whether the target is found or not
{
    int Value;
    std::vector<cv::Point2f> c_sets;
};

struct calib_sets // this basically stores the target corner_sets and point_sets
{
    std::vector<cv::Point2f> c_sets;
    std::vector<cv::Vec3f> p_sets;
};

struct calib_lists // this stores the corner_list and point_lists
{
    std::vector<std::vector<cv::Point2f>> c_lists;
    std::vector<std::vector<cv::Vec3f>> p_lists;
};

struct cam_param // this is for storing camera parameters - camera_matrix n distortion_coefficient
{
    cv::Mat cam_matrix;
    std::vector<double> dist_Coeff;
};

struct rt_vectors // this stores rotational, translational data wrt target's pose
{
    cv::Mat r_vec;
    cv::Mat t_vec;

};

Result detect_corners(cv::Mat &src, cv::Mat &dst);
calib_sets calib_points(cv::Mat &src, cv::Mat &dst, std::vector<cv::Point2f> prev_result);
cam_param calibrate_camera(cv::Mat &src, calib_lists cal_lists);
rt_vectors curr_pos_camera(calib_sets cal_sets);
int project_corners(cv::Mat &dst, calib_sets cal_sets);
void place_img(cv::Mat &dst, calib_sets cal_sets, cv::Mat img);
int feature_detector(cv::Mat &dst, calib_sets cal_sets);

#endif // TASK_H
