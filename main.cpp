// Arun Srinivasan V K (002839500), and Abhinav Anil (002889398)
// 3/20/2024
// Purpose - This is the main file that gets the i/p from the user and performs the necessary tasks

#include <iostream>
#include <opencv2/opencv.hpp>
#include <task.h>

using namespace std;

const int MIN_DATA_REQ = 5; // this is minimum number of images/data required to calibrate the camera

// this function is to calculate the point_set
std::vector<cv::Vec3f> cal_psets(Result &result) {
    std::vector<cv::Vec3f> p_sets;
    for(int i{0}; i < (int)result.c_sets.size(); i++) {
        cv::Vec3f temp(i%9, -1*(int)(i/9), 0);
        p_sets.push_back(temp);
    }

    return p_sets;
}


int main()
{ //these are some of the typical functions that we are using for the prev. projects
    cv::VideoCapture *capdev;
    capdev = new cv::VideoCapture(0);

    if(not capdev->isOpened()) {
        std::cout<<"Unable to open the camera..."<<std::endl;
        return(0);
    }

    cv::namedWindow("video", 1);
    cv::Mat frame;
    char key = 'a', temp_key;

    calib_lists data_list;
    calib_sets data_sets;
    cam_param cam_pam;

    while(true) {
        *capdev >> frame;

        if(frame.empty()) {
            std::cout<<"The frame is empty..."<<std::endl;
            break;
        }

        if(key == 'd') { // d means detect and extract target corners - task_1
            cv::Mat temp1;
            cv::cvtColor(frame, temp1, cv::COLOR_BGR2GRAY);

            Result result = detect_corners(temp1, frame);

            if(result.Value == 1) {
                data_sets.c_sets = result.c_sets;
                data_sets.p_sets = cal_psets(result);
            }
        }
        else if(key == 's') { // s is to save calibration image - task_2
            cv::Mat temp1;
            cv::cvtColor(frame, temp1, cv::COLOR_BGR2GRAY);

            calib_sets cal_sets = calib_points(temp1, frame, data_sets.c_sets);

            data_list.c_lists.push_back(cal_sets.c_sets);
            data_list.p_lists.push_back(cal_sets.p_sets);

            std::cout<<"Successfully saved..."<<std::endl;

            key = 'd';
        }
        else if(key == 'c') { // c is for calibrating the camera - task_3
            int size = data_list.c_lists.size();

            if(size < MIN_DATA_REQ) {
                std::cout<<std::endl<<"Minimum required data size is "<<MIN_DATA_REQ<<"..."<<std::endl;
                std::cout<<size-MIN_DATA_REQ<<" more data required..."<<std::endl;
            }
            else {
                cam_pam = calibrate_camera(frame, data_list);
            }

            key = 'd';
        }
        else if(key == 'p') { // p is for performing current position of the camera - task_4
            cv::Mat temp1;
            cv::cvtColor(frame, temp1, cv::COLOR_BGR2GRAY);
            Result result = detect_corners(temp1, frame);

            if(result.Value == 1) {
                data_sets.c_sets = result.c_sets;
                data_sets.p_sets = cal_psets(result);
            }

            curr_pos_camera(data_sets);
        }
        else if(key == 'j') { // j is for projecting outside corners and creating a virtual object - task_5 n task_6
            cv::Mat temp1;
            cv::cvtColor(frame, temp1, cv::COLOR_BGR2GRAY);
            Result result = detect_corners(temp1, frame);

            if(result.Value == 1) {
                data_sets.c_sets = result.c_sets;
                data_sets.p_sets = cal_psets(result);
            }

            project_corners(frame, data_sets);
        }
        else if(key == 'l') { // l is for placing an pre-captured video sequences on the target - extension
            cv::Mat temp1;
            cv::cvtColor(frame, temp1, cv::COLOR_BGR2GRAY);
            Result result = detect_corners(temp1, frame);

            if(result.Value == 1) {
                data_sets.c_sets = result.c_sets;
                data_sets.p_sets = cal_psets(result);
            }

            cv::VideoCapture *cap;
            cap = new cv::VideoCapture("/home/arun/Downloads/videoplayback.webm");
            static int video1 = 1;
            cap->set(cv::CAP_PROP_POS_FRAMES, (double)video1);
            cv::Mat img;
            *cap >> img;

            place_img(frame, data_sets, img);
            video1 += 10;

            delete cap;
        }
        else if(key == 'f') { // f is for detecting harris corners in the image and also an extension - task_7 n extension (explained briefly in task.cpp)
            cv::Mat temp1;
            cv::cvtColor(frame, temp1, cv::COLOR_BGR2GRAY);
            Result result = detect_corners(temp1, frame);

            if(result.Value == 1) {
                data_sets.c_sets = result.c_sets;
                data_sets.p_sets = cal_psets(result);
            }

            feature_detector(frame, data_sets);
        }

        cv::imshow("video", frame);
        temp_key = cv::waitKey(10);

        if(std::isalpha(temp_key)) {
            key = temp_key;
        }

        if(key == 'q') {
            break;
        }

    }

    return(0);
}
