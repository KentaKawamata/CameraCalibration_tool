#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>
#include <boost/tokenizer.hpp>

void estimate_pose(cv::Mat src){
    cv::Mat in_param(3, 3, CV_64FC1, cv::Scalar(0));
    cv::Mat dist(1, 5, CV_64FC1, cv::Scalar(0));
    cv::Mat dst;

    std::ifstream ifs("./in_param.csv");
    std::string data;


    while(getline(ifs, data)){
        std::vector<float> strvec = split(line, ",");
    }

    cv::undistort(src, dst, in_param, dist);


}

void arReader(void){
    cv::VideoCapture cap(0);

    if(!cap.isOpened()){
        std::cout << "Could not find Camera" << std::endl;
        return -1;
    }

    while(true){
        cv::Mat frame;

        cap.read(frame);
        cv::resizeWindow(frame, 640, 480);
        cv::imshow("drawDetectMarkers", frame);

        int key = cv::waitKey(100);
        if(key == 0x20){
            estimate_pose(frame);
        }
        else if(key == 27) {
            break;
        }
    }
    cap.release();
    cv::destroyAllWindows();

    return 0;
}

int main(int argc, char *argv[]) {
    arReader();
}