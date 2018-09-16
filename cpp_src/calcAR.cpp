#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <string>
#include <vector>
#include <boost/tokenizer.hpp>

#include "calcAR.h"

Calculate::Calculate() {
}

Calculate::~Calculate() {
}

void Calculate::getCameraMatrix(std::string filename, cv::Mat &param, int height, int width) {
    std::vector<float> data;
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
            dst.at<float>(row,col) = src[row][col];
        }
    }
    param = dst;
}

void Calculate::sendPoints(std::vector<cv::Vec3d> &rote, std::vector<cv::Vec3d> &t,
        std::vector<std::vector<cv::Point2f>> &corners, std::vector<int> &ids) {

    cv::Mat R(3, 3, CV_64FC1, cv::Scalar(0));
    cv::Rodrigues(rote, R);
}

void Calculate::estimatePose(cv::Mat &src){
    cv::Mat in_param(3, 3, CV_64FC1, cv::Scalar(0));
    cv::Mat dist(1, 5, CV_64FC1, cv::Scalar(0));
    getCameraMatrix("./in_param.csv", in_param, 3, 3);
    getCameraMatrix("./dist.csv", dist, 1, 5);
    cv::Mat img;

    cv::undistort(src, img, in_param, dist);

    const cv::aruco::PREDEFINED_DICTIONARY_NAME dictionary_name \
            = cv::aruco::DICT_4X4_50;
    cv::Ptr<cv::aruco::Dictionary> dictionary \
            = cv::aruco::getPredefinedDictionary(dictionary_name);

    std::vector<int> marker_ids;
    std::vector<std::vector<cv::Point2f>> marker_corners;
    cv::Ptr<cv::aruco::DetectorParameters> parameters \
            = cv::aruco::DetectorParameters::create();
    cv::aruco::detectMarkers(img, dictionary, marker_corners, marker_ids, parameters);

    std::vector<cv::Vec3d> R;
    std::vector<cv::Vec3d> t;
    cv::aruco::estimatePoseSingleMarkers(marker_corners, 150, in_param, dist, R, t);

    for(int i=0; i<marker_ids.size(); i++){
        cv::aruco::drawAxis(img, in_param, dist, R[i], t[i], 150.0);
    }

    std::vector<cv::Vec3d> R_true = {0, 45.0, 0};
    std::vector<cv::Vec3d> t_true = {0, 0, 530};
    sendPoints(R_true, t_true, marker_corners, marker_ids);
}

void Calculate::arReader(void){
    cv::VideoCapture cap(0);

    if(!cap.isOpened()){
        std::cout << "Could not find Camera" << std::endl;
        return;
    }

    while(true){
        cv::Mat frame;
        cv::Mat img;

        cap.read(frame);
        cv::resize(frame, img, 0, 640, 480);
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