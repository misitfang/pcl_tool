#pragma once

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <opencv2/opencv.hpp>
#include "solve/pnp_solver.h"
#include "camera/perspective.h"

namespace mvgpcgen
{
namespace solve
{

typedef pcl::PointCloud<pcl::PointXYZ> XYZ_PC_Type;

class BlindPnp
{
public:
    std::vector<Eigen::Vector3d> cad_3d_;
    
    BlindPnp(const std::string& pcd_file_path);
    ~BlindPnp();

    void setCameraModel(std::shared_ptr<camera::Perspective> camera);

    /**
    * @brief: 输入充电口图片上内外圆圈的像素坐标，输出初始位姿
    * @param[in] pts_2d: 图片上各个圆圈的像素坐标(左、上、右、下): 圈1内、圈1外、圈2内、圈2外、圈3外
    * @param[out] R: 输出旋转位姿 
    * @param[out] t: 输出平移位姿
    */
    void correspondingPnpSolve(
        const std::vector<std::vector<cv::Point2f>>& kps, 
        Eigen::Matrix3d& R, Eigen::Vector3d& t);
    void correspondingPnpSolve(
        const std::vector<cv::Point2f> &kps,
        Eigen::Matrix3d &R, Eigen::Vector3d &t);

    void unknownCorrespondingPnpSolve(
        const std::vector<cv::Point2f>& model_points,
        Eigen::Matrix3d& R, Eigen::Vector3d& t,bool &use_reciprocal_kdtree);

private:
    std::shared_ptr<solve::PnpSolver> pnp_solver_;
    std::shared_ptr<camera::Perspective> camera_;

    void setPnpSolver(std::shared_ptr<solve::PnpSolver> pnp_solver);

    // 充电口3d模型中各个圈的四个关键点3d坐标
    std::vector<Eigen::Vector3d> 
    kps3d_circle1_in_, kps3d_circle1_out_, kps3d_circle2_in_, kps3d_circle2_out_, kps3d_circle3_in_, kps3d_circle3_out_,
    kps3d_circle4_in_, kps3d_circle4_out_, kps3d_circle5_in_, kps3d_circle5_out_, kps3d_circle6_in_, kps3d_circle6_out_,
    kps3d_circle7_in_, kps3d_circle7_out_, kps3d_circle8_in_, kps3d_circle8_out_, kps3d_circle9_in_, kps3d_circle9_out_;

    // 充电口座模型9个圆圆心坐标
    std::vector<Eigen::Vector3d> kps3d_circles_center;
    void genKps3d();
};

}}