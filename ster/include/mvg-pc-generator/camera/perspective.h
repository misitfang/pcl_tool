#pragma once

#include <opencv2/opencv.hpp>

namespace mvgpcgen
{
namespace camera
{
class Perspective
{
public:
    Perspective(
        const int& w, const int& h, 
        const float& fx, const float& fy, 
        const float& cx, const float& cy, 
        const float& k1, const float& k2):
        w_(w), h_(h), 
        fx_(fx), fy_(fy), 
        cx_(cx), cy_(cy), 
        k1_(k1), k2_(k2)
    {
        K_ = (cv::Mat_<float>(3, 3) << fx_, cx_, 0, fy_, 0, cy_, 0, 0, 1);
        cv_dist_mat_ = (cv::Mat_<float>(5, 1) << k1_, k2_, 0, 0, 0);
    }
    
    template<class T=float>
    bool cameraToPixel(const T& x, const T& y, const T& z, T& u, T& v) const 
    {
        u = fx_*x / z + cx_;
        v = fy_*y / z + cy_;

        return (u>=0 && u<w_ && v>=0 && v<=h_);
    }

    template<class T=float>
    bool pixelToCamera(const T& u, const T&v, T& x, T& y, T& z) const 
    {
        x = (u-cx_) / fx_;
        y = (v-cy_) / fy_;
        z = 1.0;

        return true;
    }

    template<class T=float>
    void undistortPoint(const T& u_in, const T& v_in, T& u_out, T& v_out) const
    {
        cv::Mat mat(1, 2, CV_32F);

        mat.at<T>(0, 0) = u_in;
        mat.at<T>(0, 1) = v_in;
        mat = mat.reshape(2);
        cv::undistortPoints(mat, mat, K_, cv_dist_mat_, cv::Mat(), K_,
        cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 20, 1e-6));

        u_out = mat.at<T>(0, 0);
        v_out = mat.at<T>(0, 1);
    }

public:
    int w_, h_;
    float fx_, fy_, cx_, cy_;
    float k1_=0, k2_=0;
    cv::Mat K_;
    cv::Mat cv_dist_mat_;
};

}}