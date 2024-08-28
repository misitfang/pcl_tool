#pragma once

#include <vector>
#include <iostream>
#include <memory>
#include <eigen3/Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include "solve/g2o_types.h"

namespace mvgpcgen
{
namespace solve
{

class PnpSolver
{
public:
    PnpSolver() = delete;

    PnpSolver(const double& fx, const double& fy, const double& cx, const double& cy);

    ~PnpSolver();

    bool estimatePoseByEPNP(
        const std::vector<Eigen::Vector3d>& pts_3d, 
        const std::vector<Eigen::Vector2d>& pts_2d, 
        Eigen::Matrix3d& R, Eigen::Vector3d& t);

    void estimatePoseByGaussianNewton(
        const std::vector<Eigen::Vector3d>& pts_3d, 
        const std::vector<Eigen::Vector2d>& pts_2d, 
        Sophus::SE3d& pose, 
        int iterations=30, 
        double err_tol=2.0
        );

    void estimatePoseByGaussianNewton(
        const std::vector<Eigen::Vector3d>& pts_3d, 
        const std::vector<Eigen::Vector2d>& pts_2d, 
        Sophus::SE3d& pose, 
        bool use_reciprocal_kdtree,
        int iterations=30, 
        double err_tol=2.0
        );

    void estimatePoseByG2O(
        const std::vector<Eigen::Vector3d>& pts_3d, 
        const std::vector<Eigen::Vector2d>& pts_2d,
        Sophus::SE3d& pose);
private:
    double fx_, fy_, cx_, cy_;
    Eigen::Matrix3d K_;

    pcl::PointCloud<pcl::PointXY>::Ptr cloud_2d_;
    pcl::PointCloud<pcl::PointXY>::Ptr cloud_2d_reciprocal_;
    pcl::KdTreeFLANN<pcl::PointXY> kd_tree_2d_;
    pcl::KdTreeFLANN<pcl::PointXY> kd_tree_2d_reciprocal_;

    bool initKDTree2d(const std::vector<Eigen::Vector2d>& pts_2d);

    bool initKDTree3d(const std::vector<Eigen::Vector3d>& pts_3d,Sophus::SE3d& pose);


    Eigen::Matrix<double, 2, 6> computeJ(const Eigen::Vector3d& pt_cam);

    Eigen::Vector2d camToPixel(const Eigen::Vector3d& pt_cam);

    // EPNP relative
    void selectControlPoints(const std::vector<Eigen::Vector3d>& pts3d, std::vector<Eigen::Vector3d>& control_points);

    void computeHomogeneousBarycentricCoordinates(
    const std::vector<Eigen::Vector3d>& pts3d, 
    const std::vector<Eigen::Vector3d>& control_points, 
    std::vector<Eigen::Vector4d>& hb_coordinates);

    void constructM(
    const std::vector<Eigen::Vector4d>& hb_coordinates, 
    const std::vector<Eigen::Vector2d>& pts2d, 
    Eigen::MatrixXd& M);

    void getFourEigenVectors(const Eigen::MatrixXd &M, Eigen::Matrix<double, int(12), int(4)> &eigen_vectors);

    void computeL(const Eigen::Matrix<double, int(12), int(4)> &eigen_vectors, Eigen::Matrix<double, int(6), int(10)> &L);

    void computeRho(const std::vector<Eigen::Vector3d> &control_points, Eigen::Matrix<double, 6, 1> &rho);

    void computeCameraControlPoints(
    const Eigen::Matrix<double, int(12), int(4)> &eigen_vectors, 
    const Eigen::Vector4d &betas, 
    std::vector<Eigen::Vector3d> &camera_control_points);

    bool isGoodBetas(const std::vector<Eigen::Vector3d> &camera_control_points);

    void solveBetaN2(
    const Eigen::Matrix<double, 12, 4> &eigen_vectors, 
    const Eigen::Matrix<double, int(6), int(10)> &L, 
    const Eigen::Matrix<double, 6, 1> &rho, 
    Eigen::Vector4d &betas);

    void optimizeBeta(
    const Eigen::Matrix<double, int(6), int(10)> &L, 
    const Eigen::Matrix<double, int(6), int(1)> &rho, 
    Eigen::Vector4d &betas);

    void solveBetaN3(
    const Eigen::Matrix<double, 12, 4> &eigen_vectors, 
    const Eigen::Matrix<double, int(6), int(10)> &L, 
    const Eigen::Matrix<double, int(6), int(1)> &rho, 
    Eigen::Vector4d &betas);

    void solveBetaN4(
    const Eigen::Matrix<double, 12, 4> &eigen_vectors, 
    const Eigen::Matrix<double, int(6), int(10)> &L, 
    const Eigen::Matrix<double, int(6), int(1)> &rho, 
    Eigen::Vector4d &betas);

    void rebuiltPts3dCamera(
    const std::vector<Eigen::Vector3d> &camera_control_points, 
    const std::vector<Eigen::Vector4d> &hb_coordinates, 
    std::vector<Eigen::Vector3d> &pts3d_camera);

    void computeRt(
    const std::vector<Eigen::Vector3d> &pts3d_camera, 
    const std::vector<Eigen::Vector3d> &pts3d_world, 
    Eigen::Matrix3d &R, Eigen::Vector3d &t);

    double reprojectionError(
    const std::vector<Eigen::Vector3d> &pts3d_world,
    const std::vector<Eigen::Vector2d> &pts2d,
    const Eigen::Matrix3d &R,
    const Eigen::Vector3d &t);
};

}}