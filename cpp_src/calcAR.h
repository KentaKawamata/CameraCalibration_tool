#ifndef CALCAR_H
#define CALCAR_H

#include <string>
#include <vector>
#include <opencv2/opencv.hpp>

class Calculate{

    public:
        //Calculate();
        //~Calculate();
        void getCameraMatrix(std::string, cv::Mat &, int, int);
        void calcPos(float, float, cv::Mat &, std::vector<float> &,
                std::vector<float> &);
        void sendPoints(std::vector<float> &, std::vector<float> &,
                std::vector<std::vector<cv::Point2f>> &, std::vector<int> &);
        void estimatePose(cv::Mat &);
        void arReader();
};

#endif // CALCAR_H