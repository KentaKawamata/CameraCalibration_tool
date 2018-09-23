#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/video.hpp>
#include <opencv2/videoio.hpp>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <boost/tokenizer.hpp>

#include "calcAR.h"

void Calculate::getCameraMatrix(std::string filename, cv::Mat &param, int height, int width) {
    std::vector<std::vector<float>> src;
    cv::Mat dst(height, width, CV_64FC1, cv::Scalar(0));
    std::string line;
    float num;

    std::ifstream ifs(filename);
    if(!ifs){
        std::cout << "Could not found file " << filename << std::endl;
        return;
    }

    while(std::getline(ifs, line)){
       boost::tokenizer<boost::escaped_list_separator<char>> tokens(line);
       std::vector<float> data;

       for(const std::string& token : tokens){
           num = std::stof(token);

           data.push_back(num);
       }
       src.push_back(data);
    }

    if(src.size()!=height || src.front().size()!=width){
        std::cout << "Different number of parameters" << std::endl;
        return;
    }

    for(int row=0; row<height; row++){
        for(int col=0; col<width; col++){
            param.at<float>(row,col) = src[row][col];
        }
    }
    //param = dst;
}

void Calculate::calcPos(float x, float y,
        cv::Mat &R, std::vector<float> &t, std::vector<float> &r, cv::Mat &in_param) {

    //cv::Mat in_param(cv::Mat_<float>(3,3));
    //getCameraMatrix("./in_param.csv", in_param, 3, 3);
    float x0 = t[0];
    float y0 = t[1];
    float z0 = t[2];

    //std::cout << x << ", " << y << std::endl;
    //std::cout << R << std::endl << std::endl;

    //in_param.at<float>(0,0) = 640;
    //in_param.at<float>(1,1) = 640;

    x = x / in_param.at<float>(0,0);
    y = y / in_param.at<float>(1,1);

    float vec[3];
    vec[0] = R.at<float>(0,0) + R.at<float>(0,1)*x + R.at<float>(0,2)*y;
    vec[1] = R.at<float>(1,0) + R.at<float>(1,1)*x + R.at<float>(1,2)*y;
    vec[2] = R.at<float>(2,0) + R.at<float>(2,1)*x + R.at<float>(2,2)*y;

    float length;
    float theta;

    if(vec[2]<0){
        x = -vec[0] / vec[2] * z0 + x0;
        y = vec[1] / vec[2] * z0 - y0;
        length = std::sqrt(x*x + y*y);
        theta = std::atan2(y, x);
        r[0] = length * std::cos(theta);
        r[1] = length * std::sin(theta);
    } else {
        x = vec[0];
        y = vec[1];
        length = 10000.0;
        theta = std::atan2(y, x);
        r[0] = length * std::cos(theta);
        r[1] = length * std::sin(theta);
    }
    std::cout << r[0] << ", " << r[1] << std::endl;
}

void Calculate::sendPoints(std::vector<float> &rote, std::vector<float> &t,
        std::vector<std::vector<cv::Point2f>> &corners, std::vector<int> &ids, cv::Mat &param) {

    cv::Mat R(cv::Mat_<float>(3,3));
    cv::Rodrigues(rote, R);

    std::vector<float> x;
    std::vector<float> y;
    float row;
    for(int i=0; i<ids.size(); i++){
        if(ids[i]==0 || ids[i]==2){
            x.push_back(corners[i][0].x);
            //row = float(480.0) - corners[i][0].y;
            row = float(240.0) - corners[i][0].y;
            y.push_back(row);
        }
    }

    std::vector<std::vector<float>> r{{0, 0}, {0, 0}};
    for(int i=0; i<x.size(); i++){
        calcPos(x[i], y[i], R, t, r[i], param);
    }

    float strenge_x;
    float strenge_y;
    float distance;

    strenge_x = float(std::pow((r[0][0] - r[1][0]), 2));
    strenge_y = float(std::pow((r[0][1] - r[1][1]), 2));
    distance = std::sqrt(strenge_x + strenge_y);
    std::cout << "distance = " << distance << std::endl;
}

void Calculate::estimatePose(cv::Mat &src){
    //cv::Mat in_param(3, 3, CV_64F, cv::Scalar(0));
    cv::Mat in_param(cv::Mat_<float>(3,3));
    //cv::Mat dist(1, CV_64F, 5, cv::Scalar(0));
    cv::Mat dist(cv::Mat_<float>(1,5));
    getCameraMatrix("./in_param.csv", in_param, 3, 3);
    getCameraMatrix("./dist.csv", dist, 1, 5);

    std::cout << in_param << std::endl << std::endl;
    cv::Mat img;

    cv::Mat MapX, MapY;
    cv::Mat mapR = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat NewInParam = cv::getOptimalNewCameraMatrix(in_param, dist, src.size(), 0);
    cv::initUndistortRectifyMap(in_param, dist, mapR, NewInParam, src.size(), CV_32FC1, MapX, MapY);
    cv::remap(src, img, MapX, MapY, cv::INTER_AREA);

    //cv::undistort(src, img, in_param, dist);
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);

    std::vector<int> marker_ids;
    std::vector<std::vector<cv::Point2f>> marker_corners;
    cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
    cv::aruco::detectMarkers(img, dictionary, marker_corners, marker_ids, parameters);
    cv::aruco::drawDetectedMarkers(img, marker_corners, marker_ids, cv::Scalar(0, 255, 0));

    std::cout << NewInParam << std::endl << std::endl;

    std::vector<cv::Vec3d> R;
    std::vector<cv::Vec3d> t;
    cv::aruco::estimatePoseSingleMarkers(marker_corners, 50, NewInParam, dist, R, t);
    std::cout << "--------test---------" << std::endl;

    for(int i=0; i<marker_ids.size(); i++){
        cv::aruco::drawAxis(img, NewInParam, dist, R[i], t[i], 50.0);
    }

    std::vector<float> R_true = {0, 45.0, 0};
    std::vector<float> t_true = {0, 0, 510};
    sendPoints(R_true, t_true, marker_corners, marker_ids, NewInParam);

    cv::imshow("drawDetectedMarkers", img);
    if(cv::waitKey(0)==27){
      cv::imwrite("AR.png", img);
      cv::destroyAllWindows();
    }
}

void Calculate::arReader(){
    cv::VideoCapture cap("/dev/video0");

    if(!cap.isOpened()){
        std::cout << "Could not find Camera" << std::endl;
        return;
    }

    while(true){
        cv::Mat frame;
        cv::Mat img;

        cap >> frame;
        //cv::resize(frame, img, cv::Size(640, 480));
        cv::resize(frame, img, cv::Size(320, 240));
        cv::imshow("drawDetectMarkers", img);

        int key = cv::waitKey(100);
        if(key == 0x20){
            estimatePose(img);
        }
        else if(key == 27) {
            break;
        }
    }
    cap.release();
    cv::destroyAllWindows();

}

int main(int argc, char *argv[]) {
    Calculate calc;
    calc.arReader();

    return 0;
}
