#pragma once

#include <opencv2/opencv.hpp>
#include "tracking.h"
#include "data/frame.h"

namespace mvgpcgen
{
class system
{
public:
    system();

    ~system();

    void feed_stereo_frame( 
        const double& ts, 
        const cv::Mat& left_img, const cv::Mat& right_img, 
        const cv::Mat& left_mask, const cv::Mat& right_mask);

private:
    tracking* tracker_ = nullptr;
};
}