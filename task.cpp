// Arun Srinivasan V K (002839500), and Abhinav Anil (002889398)
// 3/20/2024
// Purpose - This file does all the task and extensions mentioned in the assignment

#include "task.h"
#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>
#include <cmath>

int chess_rows = 9;
int chess_cols = 6;
std::string file_path = "/home/arun/Pictures/path_to_calibration_file";

template <typename T>
void print_vector(std::vector<T> vec) {
    for(auto& coe: vec) {
        std::cout<<coe<<"   ";
    }
    std::cout<<std::endl<<std::endl;

    return ;
}

//Result, calib_sets, calib_lists, cam_param, rt_vectors - structs, please refer task.h

// d means detect and extract target corners - task_1
// btw it can detect multiple targets - extenstion_1
// we're able to detect multiple targets by placing a blank image on top of each target and finding out the other one
// this is defenitely not the best method, but yeah this is wht we've got
Result detect_corners(cv::Mat &src, cv::Mat &dst) {
    Result result;
    cv::Size patternSize(chess_rows, chess_cols);
    std::vector<cv::Point2f> corner_sets;
    bool pattern_found  = true;

    while(true) {
        pattern_found = cv::findChessboardCorners(src, patternSize, corner_sets);

        if(pattern_found == 1) {
            cv::TermCriteria criteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 30, 0.01);
            cv::cornerSubPix(src, corner_sets, cv::Size(5, 5), cv::Size(-1, -1), criteria);
        }

        cv::drawChessboardCorners(dst, patternSize, corner_sets, pattern_found);
        if(pattern_found == false) {
            break;
        }
        else {
            result.Value = pattern_found;
            result.c_sets = corner_sets;
        }

        corner_sets.clear();

        std::vector<cv::Point> temp;

        temp.push_back(corner_sets[0]);
        temp.push_back(corner_sets[chess_rows - 1]);
        temp.push_back(corner_sets[chess_rows*(chess_cols - 1)]);
        temp.push_back(corner_sets[chess_cols*chess_rows -1]);

        std::vector<std::vector<cv::Point>> polygons = {temp};

        cv::fillPoly(src, polygons, cv::Scalar(0));
    }

    if(result.c_sets.empty()) {
        result.Value = pattern_found;
        result.c_sets = corner_sets;
    }

    return result;
}

// s is to save calibration image - task_2
// code is self explanatory
calib_sets calib_points(cv::Mat &src, cv::Mat &dst, std::vector<cv::Point2f> prev_result) {
    Result result = detect_corners(src, dst);
    calib_sets cal_set;

    if(result.Value == 0) {
        result.c_sets = prev_result;
    }

    cal_set.c_sets = result.c_sets;

    for(int i{0}; i < (int)result.c_sets.size(); i++) {
        cv::Vec3f temp(i%chess_rows, -1*(int)(i/chess_rows), 0);
        cal_set.p_sets.push_back(temp);
    }

    return cal_set;
}

// c is for calibrating the camera - task_3
// code is self explanatory and does the same job as mentioned in the task
cam_param calibrate_camera(cv::Mat &src, calib_lists cal_lists) {
    double data[] = {1, 0, (double)src.cols/2, 0, 1, (double)src.rows/2, 0, 0, 1};
    cv::Mat cam_matrix(3, 3, CV_64FC1, data);
    std::vector<double> distCoeffs;
    std::vector<cv::Vec3d> rvecs, tvecs;

    std::cout<<std::endl<<"---------Parameters before calibration---------"<<std::endl;

    std::cout<<std::endl<<"Camera_matrix: "<<std::endl<<cam_matrix<<std::endl<<std::endl<<std::endl;

    std::cout<<"Distortion_Coefficient: "<<std::endl;
    print_vector(distCoeffs);

    double rms = cv::calibrateCamera(cal_lists.p_lists, cal_lists.c_lists, cv::Size(src.rows, src.cols),
                                     cam_matrix, distCoeffs, rvecs, tvecs, cv::CALIB_FIX_ASPECT_RATIO);

    std::cout<<std::endl<<"---------Parameters after calibration---------"<<std::endl;

    std::cout<<std::endl<<"Camera_matrix: "<<std::endl<<cam_matrix<<std::endl<<std::endl<<std::endl;

    std::cout<<"Distortion_Coefficient: "<<std::endl;
    print_vector(distCoeffs);

    std::cout<<"RMS re-proj. error: "<<std::endl<<rms<<std::endl<<std::endl;
    std::cout<<"Rotation Vectors: "<<std::endl;
    print_vector(rvecs);

    std::cout<<"Translational vectors: "<<std::endl;
    print_vector(tvecs);

    cam_param cam_pam;
    cam_pam.cam_matrix = cam_matrix.clone();
    cam_pam.dist_Coeff = distCoeffs;

    cv::FileStorage fs(file_path, cv::FileStorage::WRITE);
    fs << "camera_matrix" << cam_pam.cam_matrix;
    fs << "distortion_matrix" << cam_pam.dist_Coeff;

    return cam_pam;
}

// p is for performing current position of the camera - task_4
// code is self explanatory
rt_vectors curr_pos_camera(calib_sets cal_sets) {
    rt_vectors rt_vecs;
    cam_param cam_pam;

    cv::FileStorage fs(file_path, cv::FileStorage::READ);
    fs["camera_matrix"] >> cam_pam.cam_matrix;
    fs["distortion_matrix"] >> cam_pam.dist_Coeff;

    cv::solvePnP(cal_sets.p_sets, cal_sets.c_sets, cam_pam.cam_matrix, cam_pam.dist_Coeff, rt_vecs.r_vec, rt_vecs.t_vec);

    // std::cout<<std::endl<<"---------Current position of the camera---------"<<std::endl;

    // std::cout<<std::endl<<"Translational vector: "<<std::endl<<rt_vecs.t_vec<<std::endl<<std::endl<<std::endl;

    // std::cout<<std::endl<<"Rotational vector: "<<std::endl<<rt_vecs.r_vec<<std::endl<<std::endl<<std::endl;

    return rt_vecs;
}

// j is for projecting outside corners and creating a virtual object - task_5 n task_6 n extension_2
int project_corners(cv::Mat &dst, calib_sets cal_sets) {
    std::vector<cv::Point2f> img_points;
    cam_param cam_pam;
    rt_vectors rt_vecs = curr_pos_camera(cal_sets);
    std::vector<cv::Vec3f> p_sets;
    std::vector<cv::Vec3b> color_val;


    // this is the code to create virtual object - task_6 n extension_2
    // ik, more than the object the color for it looks complicated, but after seeing the result I felt like everthing is worth it.
    // thanks prof. and team
    for(int i{0}; i < 360; i++) {
        for(int j{200}; j < 500; j++) {
            //this object is a combination of circle and a sine wave
            p_sets.push_back(cv::Vec3f(4.0 + std::cos((i/180.0)*M_PI) + std::sin((j/100.0)*3.0*M_PI)/4.0, -3.0 + std::sin((i/180.0)*M_PI), j/100.0));

            cv::Vec3b color(255 - 200*std::abs(std::sin(M_PI/2.0 + i*M_PI/360.0)) - 50*std::abs(std::sin((j/100.0)*3.0*M_PI)),
                            255 - 200*std::abs(std::sin(-3.0*M_PI/4.0 + i*M_PI/360.0)) - 50*std::abs(std::sin((j/100.0)*3.0*M_PI)),
                            255 - 200*std::abs(std::sin(3.0*M_PI/4.0 + i*M_PI/360.0)) - 50*std::abs(std::sin((j/100.0)*3.0*M_PI)));
            color_val.push_back(color);

        }
    }

    cv::FileStorage fs(file_path, cv::FileStorage::READ);
    fs["camera_matrix"] >> cam_pam.cam_matrix;
    fs["distortion_matrix"] >> cam_pam.dist_Coeff;

    cv::projectPoints(p_sets, rt_vecs.r_vec, rt_vecs.t_vec, cam_pam.cam_matrix, cam_pam.dist_Coeff, img_points);

    for(int i{0}; i < (int)img_points.size(); i++) {
        cv::circle(dst, img_points[i], 4, color_val[i], -1);
    }

    p_sets.clear();
    img_points.clear();

    // code for projecting outside corners - task_5
    p_sets.push_back(cv::Vec3f(0, 0, 2));
    p_sets.push_back(cv::Vec3f(8, 0, 2));
    p_sets.push_back(cv::Vec3f(0, -5, 2));
    p_sets.push_back(cv::Vec3f(8, -5, 2));

    cv::projectPoints(p_sets, rt_vecs.r_vec, rt_vecs.t_vec, cam_pam.cam_matrix, cam_pam.dist_Coeff, img_points);

    for(int i{0}; i < (int)img_points.size(); i++) {
        cv::circle(dst, img_points[i], 4, cv::Scalar(0, 255, 0), -1);
    }

    return(0);
}

// l is for placing an pre-captured video sequences on the target - extension_3
void place_img(cv::Mat &dst, calib_sets cal_sets, cv::Mat img) {
    if(img.empty()) {
        img = cv::imread("/home/arun/Downloads/OIP.jpeg"); // this is a 404 error image. It'll be displayed if the video is unavailable
    }

    std::vector<cv::Point2f> img_points;
    cam_param cam_pam;
    rt_vectors rt_vecs = curr_pos_camera(cal_sets);

    std::vector<cv::Vec3f> p_sets_temp;
    p_sets_temp.push_back(cv::Vec3f(-1, 1, 2));
    p_sets_temp.push_back(cv::Vec3f(-1, -6, 2));
    p_sets_temp.push_back(cv::Vec3f(9, 1, 2));
    p_sets_temp.push_back(cv::Vec3f(9, -6, 2));

    cv::FileStorage fs(file_path, cv::FileStorage::READ);
    fs["camera_matrix"] >> cam_pam.cam_matrix;
    fs["distortion_matrix"] >> cam_pam.dist_Coeff;

    cv::projectPoints(p_sets_temp, rt_vecs.r_vec, rt_vecs.t_vec, cam_pam.cam_matrix, cam_pam.dist_Coeff, img_points);

    std::vector<cv::Point2f> img_sets;
    img_sets.push_back(cv::Point2f(0, 0));
    img_sets.push_back(cv::Point2f(img.rows-1, 0));
    img_sets.push_back(cv::Point2f(0, img.cols-1));
    img_sets.push_back(cv::Point2f(img.rows-1, img.cols-1));

    cv::Mat perspective_matrix = cv::getPerspectiveTransform(img_sets, img_points);


    for(int i{0}; i < img.rows; i++) {
        for(int j{0}; j < img.cols; j++) {
            cv::Mat src_pts = (cv::Mat_<double>(3, 1) << i, j, 1);
            cv::Mat dst_pts = perspective_matrix*src_pts;

            float dstY = dst_pts.at<double>(0, 0)/dst_pts.at<double>(2, 0);
            float dstX = dst_pts.at<double>(1, 0)/dst_pts.at<double>(2, 0);

            if(dstX >= 0 and dstX < dst.rows and dstY >= 0 and dstY < dst.cols) {
                dst.at<cv::Vec3b>(dstX, dstY) = img.at<cv::Vec3b>(i, j);
            }

        }
    }

    std::vector<cv::Vec3f> p_sets;
    p_sets.push_back(cv::Vec3f(-1, 1, 2)); //0 - index
    p_sets.push_back(cv::Vec3f(-1, -6, 2)); //1
    p_sets.push_back(cv::Vec3f(9, 1, 2)); //2
    p_sets.push_back(cv::Vec3f(9, -6, 2)); //3
    p_sets.push_back(cv::Vec3f(-1, 1, 0)); //4
    p_sets.push_back(cv::Vec3f(-1, -6, 0)); //5
    p_sets.push_back(cv::Vec3f(9, 1, 0)); //6
    p_sets.push_back(cv::Vec3f(9, -6, 0)); //7

    img_points.clear();

    cv::projectPoints(p_sets, rt_vecs.r_vec, rt_vecs.t_vec, cam_pam.cam_matrix, cam_pam.dist_Coeff, img_points);

    for(int i{0}; i < (int)img_points.size(); i++) {
        cv::circle(dst, img_points[i], 4, cv::Scalar(0, 255, 0), -1);
    }

    // these are the 8 lines in the cuboid
    cv::line(dst, img_points[0], img_points[1], cv::Scalar(0, 255, 0));
    cv::line(dst, img_points[0], img_points[2], cv::Scalar(0, 255, 0));
    cv::line(dst, img_points[0], img_points[4], cv::Scalar(0, 255, 0));

    cv::line(dst, img_points[7], img_points[3], cv::Scalar(0, 255, 0));
    cv::line(dst, img_points[7], img_points[5], cv::Scalar(0, 255, 0));
    cv::line(dst, img_points[7], img_points[6], cv::Scalar(0, 255, 0));

    cv::line(dst, img_points[1], img_points[5], cv::Scalar(0, 255, 0));
    cv::line(dst, img_points[4], img_points[5], cv::Scalar(0, 255, 0));
    cv::line(dst, img_points[4], img_points[6], cv::Scalar(0, 255, 0));
    cv::line(dst, img_points[2], img_points[6], cv::Scalar(0, 255, 0));
    cv::line(dst, img_points[2], img_points[3], cv::Scalar(0, 255, 0));
    cv::line(dst, img_points[1], img_points[3], cv::Scalar(0, 255, 0));

    return ;
}

// f is for detecting harris corners in the image and also an extension - task_7 n extension_4
// a small planetory system is made as an extension here
int feature_detector(cv::Mat &dst, calib_sets cal_sets) {
    std::vector<cv::Point2f> img_points;
    cam_param cam_pam;
    rt_vectors rt_vecs = curr_pos_camera(cal_sets);
    std::vector<cv::Vec3f> p_sets;

    static int pos = 0;
    p_sets.push_back(cv::Vec3f(4, -3, 3));
    p_sets.push_back(cv::Vec3f(4, -3, 2));

    for(int i{0}; i < 360; i++) {
        p_sets.push_back(cv::Vec3f(4.0 + 4.0*std::cos((i/180.0)*M_PI), -3.0 + 4.0*std::sin((i/180.0)*M_PI), 3));
    }

    cv::FileStorage fs(file_path, cv::FileStorage::READ);
    fs["camera_matrix"] >> cam_pam.cam_matrix;
    fs["distortion_matrix"] >> cam_pam.dist_Coeff;

    cv::projectPoints(p_sets, rt_vecs.r_vec, rt_vecs.t_vec, cam_pam.cam_matrix, cam_pam.dist_Coeff, img_points);

    //code for the revolving planetory system - extension_4
    //in this code i'd say the better way is to put varying radius rather than a constat one
    // then i thought a much better way would be make the sphere out of mutiple circle (consisting of multiple points) and add a varying color like in extension_3
    // we did try this, but the problem was fps was dropping heavily - one method we thought of doing is hyper-threading, if there is anyother method please mention them

    //the computation/ complexity will be going on increasing, that's why we chose this. but we've included the varying radius code

    // float euc_dist = std::sqrt((img_points[0].x - img_points[1].x)*(img_points[0].x - img_points[1].x) +
    //                            (img_points[0].y - img_points[1].y)*(img_points[0].y - img_points[1].y));

    cv::circle(dst, img_points[0], 40, cv::Scalar(35, 35, 150), -1);

    for(int j{0}; j < 360; j++) {
        cv::circle(dst, img_points[2 + j], 4, cv::Scalar(35, 35, 35), -1);
    }

    cv::circle(dst, img_points[2 + (int)pos], 15, cv::Scalar(35, 35, 100), -1);

    pos += 2;
    pos = pos%360;

    cv::Mat corners;
    cv::Mat sample_img;
    cv::cvtColor(dst, sample_img, cv::COLOR_RGB2GRAY);

    // harris corner task_7
    cv::cornerHarris(sample_img, corners, 3, 3, 0.1);

    cv::Mat dst_norm, dst_norm_scaled;
    cv::normalize(corners, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat());
    cv::convertScaleAbs(dst_norm, dst_norm_scaled);

    std::vector<cv::Point> data;

    for(int i = 0; i < dst_norm.rows; i++) {
        for(int j = 0; j < dst_norm.cols; j++ ) {
            if((int)dst_norm.at<float>(i,j) > 180) {
                data.push_back(cv::Point(j, i));
            }
        }
    }

    if((int)data.size() > 1000) {
        return(0);
    }
    for(int i{0}; i < (int)data.size(); i++) {
        cv::circle(dst, data[i], 5, cv::Scalar(0, 255, 0), -1);
    }

    return(0);
}

//extension_5
// the re-projection error from a recently brought (a month old) phone is around 0.226 (average of 3)
// the re-projection error from a old (a year old) phone is around 0.296 (average of 3)
// the re-projection error from a 6 month old phone is 0.249 (aveage of 3)

// from this we can infer that the re-proj error is directly proportional to age of the phone.
