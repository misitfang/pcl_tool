#pragma once

#include <vector>
#include <iostream>
#include <Eigen/Dense>

namespace mvgpcgen
{
namespace util
{

class PoseTrans
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static bool isRotationMatrix(const Eigen::Matrix3d& R);

    static void eulerAngleToMat3d(const std::vector<double>& euler_ang, Eigen::Matrix3d& R);
    static void eulerAngleToMat3d(const Eigen::Vector3d& euler_ang, Eigen::Matrix3d& R);
    static void mat3dToEulerAngles(const Eigen::Matrix3d& R, std::vector<double>& euler_angle);
    static void mat3dToEulerAngles(const Eigen::Matrix3d& R, Eigen::Vector3d& euler_angle);

private:
};

}} 