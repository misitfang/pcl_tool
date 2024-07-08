#pragma once

#include <yaml-cpp/yaml.h>
#include "feature/orb/orb_extractor.h"
#include "data/frame.h"

namespace mvgpcgen
{
class tracking
{
public:
    tracking(const std::string& config_file_path);

    ~tracking();
    
    void track_stereo_image(
        const double timestamp, 
        const cv::Mat& left_img_rect, const cv::Mat& right_img_rect, 
        const cv::Mat& mask0, const cv::Mat& mask1);

public:
    YAML::Node yaml_node_;

    feature::orb_extractor* orb_extra_left_ = nullptr;
    feature::orb_extractor* orb_extra_right_ = nullptr;

    data::frame cur_frame_;

    Eigen::Matrix4d T_rl_;
};
} 
