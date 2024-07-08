#pragma once

#include <vector>
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <pcl/kdtree/kdtree_flann.h>
#include "solve/g2o_types.h"

namespace mvgpcgen
{
namespace solve
{

class ICPSolver
{
public:
    ICPSolver() = default;

    /**
    * @brief: 已知两组匹配的3d点，计算出相对运动R|t
    */
    void estimatePoseBySVD(
        const std::vector<Eigen::Vector3d>& pts3d_1, const std::vector<Eigen::Vector3d>& pts3d_2,
        Eigen::Matrix3d& R, Eigen::Vector3d& t);

private:

};

}}