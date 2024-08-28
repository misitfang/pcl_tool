#pragma once

#include "types.h"
#include <sophus/se3.hpp>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/solver.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/linear_solver_dense.h>


namespace mvgpcgen
{
namespace solve
{

/// 位姿顶点: 数据维度6, 数据类型Sophus::SE3
class VertexPose: public g2o::BaseVertex<6, Sophus::SE3d>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    // 重置
    virtual void setToOriginImpl() override { _estimate=Sophus::SE3d(); }

    // 更新：SE3上的左乘模型
    virtual void oplusImpl(const double* update) override 
    {
        Vector6d update_eigen;
        update_eigen << update[0], update[1], update[2], update[3], update[4], update[5];
        _estimate = Sophus::SE3d::exp(update_eigen) * _estimate;
    }

    // 存盘和读盘
    virtual bool read(std::istream& in) override {return true; }
    virtual bool write(std::ostream& out) const override {return true; }
};

/// 路标顶点：数据维度3，数据类型Eigen::Vec3d
class VertexXYZ: public g2o::BaseVertex<3, Eigen::Vector3d>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    // 重置
    virtual void setToOriginImpl() override { _estimate=Eigen::Vector3d::Zero(); }

    // 更新
    virtual void oplusImpl(const double* update) override 
    {
        _estimate[0] += update[0];
        _estimate[1] += update[1];
        _estimate[2] += update[2];
    }

    // 存盘和读盘
    virtual bool read(std::istream& in) override { return true; }
    virtual bool write(std::ostream& out) const override { return true; }
};

/// 仅估计位姿的一元边(连接边的两个顶点均是位姿顶点): 观测值维度2，观测值数据类型Eigen::Vec2d,
/// 连接顶点数据类型VertexPose
class EdgeProjectionPoseOnly: public g2o::BaseUnaryEdge<2, Eigen::Vector2d, VertexPose>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    EdgeProjectionPoseOnly(const Eigen::Vector3d& pos, const Eigen::Matrix3d& K):_pos3d(pos), _K(K) {}

    virtual void computeError() override 
    {
        // 取出边所连接顶点的估计值
        const VertexPose* v = static_cast<VertexPose *>(_vertices[0]);
        Sophus::SE3d T = v->estimate();

        // 根据位姿估计值计算 像素坐标估计值
        Eigen::Vector3d pos_pixel = _K * (T * _pos3d);
        pos_pixel /= pos_pixel[2];

        // 与测量值计算误差
        _error = _measurement - pos_pixel.head<2>();
    }

    // 计算每条边相对于顶点的雅可比
    virtual void linearizeOplus() override 
    {
        const VertexPose* v = static_cast<VertexPose *>(_vertices[0]);
        Sophus::SE3d T = v->estimate();
        Eigen::Vector3d pos_cam = T * _pos3d;

        double fx = _K(0, 0);
        double fy = _K(1, 1);
        double X = pos_cam[0];
        double Y = pos_cam[1];
        double Z = pos_cam[2];
        double ZInv = 1.0 / (Z+1e-18);
        double ZInv2 = ZInv * ZInv;

        _jacobianOplusXi << -fx * ZInv, 0, fx * X * ZInv2, fx * X * Y * ZInv2, -fx - fx * X * X * ZInv2, fx * Y * ZInv, 
                            0, -fy * ZInv, fy * Y * ZInv2, fy + fy * Y * Y * ZInv2, -fy * X * Y * ZInv2, -fy * X * ZInv;
    }

    virtual bool read(std::istream& in){return true; }
    virtual bool write(std::ostream& out) const override {return true; }

private:
    Eigen::Vector3d _pos3d;
    Eigen::Matrix3d _K;
};

/// 带有路标和位姿的二元边(连接边的两个顶点是位姿顶点和路标顶点): 观测值维度2，观测值数据类型Eigen::Vec2d,
/// 连接顶点数据类型VertexPose, VertexXYZ
class EdgeProjection: public g2o::BaseBinaryEdge<2, Eigen::Vector2d, VertexPose, VertexXYZ>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    // 构造函数传入相机内外参
    EdgeProjection(const Eigen::Matrix3d& K, const Sophus::SE3d& cam_ext):_K(K), _cam_ext(cam_ext){}

    virtual void computeError() override 
    {
        // 取出边所连接位姿顶点的估计值
        const VertexPose* v0 = static_cast<VertexPose *>(_vertices[0]);

        // 取出边所连接路标顶点的估计值
        const VertexXYZ* v1 = static_cast<VertexXYZ *>(_vertices[1]);

        // 计算像素坐标估计值
        Sophus::SE3d T = v0->estimate();
        Eigen::Vector3d pos_pixel = _K * (_cam_ext * (T * v1->estimate()));
        pos_pixel /= pos_pixel[2];

        // 计算误差
        _error = _measurement - pos_pixel.head<2>();
    }

    // 计算每条边相对于顶点的雅可比
    virtual void linearizeOplus() override 
    {
        const VertexPose* v0 = static_cast<VertexPose*>(_vertices[0]);
        const VertexXYZ* v1 = static_cast<VertexXYZ*>(_vertices[1]);
        Sophus::SE3d T = v0->estimate();
        Eigen::Vector3d pw = v1->estimate();
        Eigen::Vector3d pos_cam = _cam_ext * T * pw;

        double fx = _K(0, 0);
        double fy = _K(1, 1);
        double X = pos_cam[0];
        double Y = pos_cam[1];
        double Z = pos_cam[2];
        double ZInv = 1.0 / (Z + 1e-18);
        double ZInv2 = ZInv * ZInv;

        // 相对于位姿顶点的雅可比
        _jacobianOplusXi <<
                -fx * ZInv, 0, fx * X * ZInv2, fx * X * Y * ZInv2, -fx - fx * X * X * ZInv2, fx * Y * ZInv, 
                0, -fy * ZInv, fy * Y * ZInv2, fy + fy * Y * Y * ZInv2, -fy * X * Y * ZInv2, -fy * X * ZInv;

        // 相对于路标顶点的雅可比
        _jacobianOplusXj = _jacobianOplusXi.block<2, 3>(0, 0) *
                           _cam_ext.rotationMatrix() * T.rotationMatrix();
    }

    virtual bool read(std::istream& in) override { return true; }
    virtual bool write(std::ostream& out) const override { return true; }

private:
    Eigen::Matrix3d _K;
    Sophus::SE3d _cam_ext;
};

}} // end namespace