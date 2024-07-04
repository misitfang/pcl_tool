#ifndef GET_FINAL_RESULT_H
#define GET_FINAL_RESULT_H

#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>

class ResultProcess

{
public:
    ResultProcess(/* args */);
    ~ResultProcess();
    static void GetTransformationMatrix(const Eigen::Matrix3d &rotation_matrix, const Eigen::Vector3d &translation_matrix,
                                        Eigen::Matrix4d &transformation_matrix);
    static void TransToTcpCoordinate(const Eigen::Matrix4d &transformation_matrix, Eigen::Matrix<double, 6, 1> &result);
    static void GetFinalResult(const Eigen::Matrix3d &pnp_solve_R, const Eigen::Vector3d &pnp_solve_t,
                               const Eigen::Matrix3d &R_eye_in_hand, const Eigen::Vector3d &t_eye_in_hand,
                               const Eigen::Vector3d &tcp, Eigen::Matrix<double, 6, 1> &result);
};

#endif