#ifndef CALCAR_H
#define CALCAR_H

#include <string>
#include <vector>
#include <opencv2/opencv.hpp>

class Calculate{

    public:
        Calculate();
        ~Calculate();
        void getCameraMatrix(std::string, cv::Mat &, int, int);
        void calcPos(float , float , float , float);
        void sendPoints(std::vector<cv::Vec3d> &, std::vector<cv::Vec3d> &,
                std::vector<std::vector<cv::Point2f>> &, std::vector<int> &);
        void estimatePose(cv::Mat &);
        void arReader(void);
};

#endif // CALCAR_H