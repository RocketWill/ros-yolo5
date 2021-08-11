#ifndef __COLOR_MAPPING_H__
#define __COLOR_MAPPING_H__

#include <vector> 
#include "opencv2/opencv.hpp"

extern std::vector<cv::Scalar> colors = {
    cv::Scalar(195, 26, 203), // Java
    cv::Scalar(198, 97, 125), // Indigo
    cv::Scalar(43, 247, 210), // Saffron
    cv::Scalar(66, 242, 94), // Flamingo
    cv::Scalar(128, 240, 128), // Light Coral
    cv::Scalar(50, 154, 205), // yellow green
    cv::Scalar(50, 154, 205), // yellow green
    cv::Scalar(255, 0, 255), // cyan
    cv::Scalar(255, 30, 144), // dodger blue	
    cv::Scalar(238, 238, 130), // violet
};

#endif  // __COLOR_MAPPING_H__