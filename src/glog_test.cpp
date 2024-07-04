#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <glog/logging.h>

#define CURRENT_TIME() GetCurTime()
// 获取当前时间并返回格式化后的字符串
std::string GetCurTime()
{
  time_t a = time(NULL);
  tm currtime = *localtime(&a);
  std::ostringstream Tm;
  Tm.fill('0');
  Tm << "TEST"
     << currtime.tm_year + 1900
     << std::setw(2) << currtime.tm_mon + 1
     << std::setw(2) << currtime.tm_mday
     << "_"
     << std::setw(2) << currtime.tm_hour
     << std::setw(2) << currtime.tm_min
     << std::setw(2) << currtime.tm_sec
     << "_";
  return Tm.str();
}

int main()
{
  std::string log_info_path = "../data/logs/info/";
  std::string log_warn_path = "../data/logs/warn/";
  std::string log_info, log_warn;
  std::string str_time = CURRENT_TIME();
  DIR *dir;
  std::string str_command;

  if ((dir = opendir(log_info_path.c_str())) == NULL)
  {
    LOG(INFO) << "日志存放目录不存在，创建日志存放目录文件" << std::endl;
    str_command = "mkdir -p " + log_info_path;
    system(str_command.c_str()); // 存在就删除目录
  }
  // if ((dir = opendir(log_warn_path.c_str())) == NULL)
  // {
  //   LOG(INFO) << "日志存放目录文件不存在，创建日志存放目录文件" << std::endl;
  //   str_command = "mkdir -p " + log_warn_path;
  //   system(str_command.c_str()); // 存在就删除目录
  // }
  log_info = log_info_path +"Test_";
  // log_warn = log_warn_path + str_time;
  // logErrorPath = logPath + "error/";
  // logFatalPath = logPath + "fatal/";
  google::SetLogDestination(google::GLOG_INFO, log_info.c_str());
  // google::SetLogDestination(google::GLOG_WARNING, log_warn.c_str());
  google::SetLogFilenameExtension(".log");
  // google::SetLogDestination(google::GLOG_ERROR, logErrorPath.c_str());
  LOG(INFO) << "google::IsGoogleLoggingInitialized :" << google::IsGoogleLoggingInitialized() << std::endl;
  if (!google::IsGoogleLoggingInitialized())
  {
    google::InitGoogleLogging("init logs------");
    // LOG(WARNING) << "IsGoogleLoggingInitialized warning...";

    LOG(INFO) << ">>>>>>>>> init google log<<<<<<<<<<<  ";
  }
  LOG(INFO) << ">>>>>>>>>  log start <<<<<<<<<<<  ";
  FLAGS_alsologtostderr = true; // true 同时打印到终端
  FLAGS_max_log_size =10;
  Eigen::Vector4d cir_1(0.01, 0, 0, 1);
  Eigen::Vector4d tcp(-0.0250411, -0.0199086, 0.327973, 0);
  Eigen::Vector4d tcp_frame, flange_frame, cam_frame;
  Eigen::Matrix4d T_tcp_2_point;
  Eigen::Matrix4d T_cam_2_tcp;
  Eigen::Vector3d uv_1, cam_1;
  Eigen::Matrix<double, 3, 4> K_1;
  Eigen::Matrix3d K;
  K_1 << 1719.57, 0, 953.387, 0,
      0, 1720.51, 546, 0,
      0, 0, 1, 0;
  T_tcp_2_point << 0.999985, 0.00165702, -0.00558042, 0.0536954,
      -0.00149043, 0.99956, 0.0296927, -0.00915292,
      0.0056271, -0.0296839, 0.999544, 0.141547,
      0, 0, 0, 1;
  tcp_frame = T_tcp_2_point * cir_1;
  for(int i=0;i<1000000;i++){

  LOG(INFO) << "tcp_frame: " << tcp_frame[0] << " , " << tcp_frame[1] << " , " 
  << tcp_frame[2] << " , " << tcp_frame[3] << std::endl;
  flange_frame = tcp_frame + tcp;
  LOG(INFO) << "flange_frame: " << flange_frame[0] << " , " << flange_frame[1] 
      << " , " << flange_frame[2] << " , " << flange_frame[3] << std::endl;
    if (!google::IsGoogleLoggingInitialized())
  {
    
    LOG(INFO) << ">>>>>>>>> init google log<<<<<<<<<<<  ";
  }
  }

  T_cam_2_tcp << -0.027445, -0.999507, -0.01527, 0.087639,
      0.99898, -0.026876, -0.036295, -0.006377,
      0.035867, -0.016251, 0.999224, 0.081998,
      0, 0, 0, 1;
  cam_frame = T_cam_2_tcp.inverse() * flange_frame;
  LOG(INFO) << "cam_frame: " << cam_frame[0] << " , " << cam_frame[1] << " , " << cam_frame[2] << " , " << cam_frame[3] << std::endl;
  cam_frame[0] = cam_frame[0] / cam_frame[2];
  cam_frame[1] = cam_frame[1] / cam_frame[2];
  cam_frame[2] = cam_frame[2] / cam_frame[2];
  LOG(INFO) << "cam_frame: " << cam_frame[0] << " , " << cam_frame[1] << " , " << cam_frame[2] << " , " << cam_frame[3] << std::endl;

  uv_1 = K_1 * cam_frame;
  LOG(INFO) << "uv_1: " << uv_1[0] << " , " << uv_1[1] << " , " << uv_1[2] << " , " << uv_1[2] << std::endl;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filter(new pcl::PointCloud<pcl::PointXYZ>);

  pcl::io::loadPCDFile("./obpcd.pcd", *cloud_in);
  Eigen::Matrix4f rotation_T;
  rotation_T << 1, 0, 0, 0,
      -0, 0.484802, 0.874624, 0,
      0, -0.874624, 0.484802, 0,
      0, 0, 0, 1;
  pcl::transformPointCloud(*cloud_in, *cloud_out, rotation_T);
  // pcl::io::savePCDFile("./obpcd.pcd", *cloud_out);

  float m_tx, m_ty, m_tz, m_y_length, m_x_out, m_z_length, m_z_cut, m_x_in;
  m_tx = 0;
  m_ty = 1.92;
  m_tz = 0;
  m_y_length = 0.25;
  m_x_out = 0.6;
  m_z_length = 1.0;
  m_z_cut = 0.05;
  m_x_in = 0.34;

  for (size_t i = 0; i < cloud_out->size(); i++)
  {
    if (cloud_out->points[i].y < m_ty - m_y_length && cloud_out->points[i].y >= 0 && cloud_out->points[i].x <= m_x_in - m_tx && cloud_out->points[i].x >= -m_x_in - m_tx &&
        cloud_out->points[i].z <= m_z_length - m_tz && cloud_out->points[i].z > m_z_cut)
    {
#pragma omp critical
      {
        cloud_filter->push_back(cloud_out->points[i]);
        //  LOG(INFO)<<"m_candidates_out_left back-------------"<<std::endl;
      }
    }
  }

  // pcl::io::savePCDFile("./obpcd_filter.pcd", *cloud_filter);

  return 0;
}