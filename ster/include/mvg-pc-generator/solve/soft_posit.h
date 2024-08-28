#pragma once

#include <vector>
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

namespace mvgpcgen
{
namespace solve
{

class SoftPosit
{
public:
    SoftPosit() = default;

    SoftPosit(const double& fx, const double& fy, const double& cx, const double& cy);

    ~SoftPosit();

    Eigen::MatrixXd sinkhornImpl(Eigen::MatrixXd assign_mtx);


private:
    double fx_, fy_, cx_, cy_;
    Eigen::Matrix3d K_;

};

}}