#pragma once
#include "camera/perspective.h"
#include <yaml-cpp/yaml.h>

namespace mvgpcgen
{
namespace util
{
class stereo_rectifier
{
public:
    stereo_rectifier() = default;

    stereo_rectifier(camera::Perspective* camera, const YAML::Node& yaml_node);

    ~stereo_rectifier();

    void rectify(const cv::Mat& in_img_l, const cv::Mat& in_img_r, cv::Mat& out_img_l, cv::Mat& out_img_r) const;
    
private:
    // undistortion map for x-axis in left image
    cv::Mat undist_map_x_l_;
    // undistortion map for y-axis in left image
    cv::Mat undist_map_y_l_;
    // undistortion map for x-axis in right image
    cv::Mat undist_map_x_r_;
    // undistortion map for y-axis in right image
    cv::Mat undist_map_y_r_;

    cv::Mat parse_vector_as_mat(const cv::Size& shape, const std::vector<double>& vec);
};
}}