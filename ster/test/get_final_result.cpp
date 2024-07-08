#include "get_final_result.h"

void ResultProcess::GetTransformationMatrix(const Eigen::Matrix3d &rotation_matrix, const Eigen::Vector3d &translation_matrix,
                             Eigen::Matrix4d &transformation_matrix)
{

  Eigen::Isometry3d T_M3 = Eigen::Isometry3d::Identity();
  T_M3.rotate(rotation_matrix);
  T_M3.pretranslate(translation_matrix);
  transformation_matrix = T_M3.matrix();
}
void ResultProcess::TransToTcpCoordinate(const Eigen::Matrix4d &transformation_matrix, Eigen::Matrix<double, 6, 1> &result)
{
  Eigen::Affine3d Trans(transformation_matrix);
  Eigen::Matrix3d R = Trans.rotation();
  Eigen::AngleAxisd result_r;
  result_r.fromRotationMatrix(R);
  Eigen::Vector3d result_eulerangle = result_r.matrix().eulerAngles(0, 1, 2);
  Eigen::Vector3d result_t = Trans.translation();
  result(0, 0) = result_t[0];
  result(1, 0) = result_t[1];
  result(2, 0) = result_t[2];
  result(3, 0) = result_eulerangle[0]; // yaw
  result(4, 0) = result_eulerangle[1]; // pitch
  result(5, 0) = result_eulerangle[2]; // roll
}
void ResultProcess::GetFinalResult(const Eigen::Matrix3d &pnp_solve_R, const Eigen::Vector3d &pnp_solve_t,
                    const Eigen::Matrix3d &R_eye_in_hand, const Eigen::Vector3d &t_eye_in_hand,
                    const Eigen::Vector3d &tcp ,Eigen::Matrix<double, 6, 1> &result)
{ 
  Eigen::Matrix4d T_point_2_uv, T_cam_2_tcp, T_tcp_2_point;
  GetTransformationMatrix(pnp_solve_R, pnp_solve_t, T_point_2_uv);
  Eigen::Vector3d t_eye_2_tcp = t_eye_in_hand - tcp;
  GetTransformationMatrix(R_eye_in_hand, t_eye_2_tcp, T_cam_2_tcp);
  T_tcp_2_point = T_cam_2_tcp * T_point_2_uv;
  TransToTcpCoordinate(T_tcp_2_point, result);
  std::cout << "pnp求解值 T_point_2_uv=\n" << T_point_2_uv << "\n";
  std::cout << "result T_tcp_2_point=\n" << T_tcp_2_point << "\n";
  std::cout << "result in tcpcoordinate:x y z rx ry rz\n" << result << std::endl;
}