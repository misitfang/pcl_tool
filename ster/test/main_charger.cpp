#ifdef _WIN32
#include <direct.h>
#include <io.h>
#else
#include <sys/time.h>
#include <sys/uio.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <dirent.h>
#endif
#include <algorithm>
#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include "opencv2/opencv.hpp"
#include "ahpc_detector.h"
#include <stdlib.h>
#include <memory>
#include "camera/perspective.h"
#include "solve/blind_pnp.h"
#include <fstream>
#define SHOW_IMGS
#include <get_final_result.h>
#include "TYImageProc.h"
#include "../common/common.hpp"
#include "jaka_move_interface.hpp"

#include "INIReader.h"
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>

#include <types.h>
#include <sqpnp.h>

#include "robust_pose_pnp.h"
#include <get_final_result.h>

#include <glog/logging.h>


std::string currtime_,out_path;
std::vector<double> cam_K = {1920, 1080, 1719.57, 1720.51, 953.387, 546, 0.0, 0.0}; // w, h, fx, fy, cx, cy, k1, k2
Eigen::Vector3d tcp;
// Eigen::Vector3d tcp(-0.026041, -0.019909, 0.330973);
Eigen::Matrix3d R_eye_in_hand;
Eigen::Matrix3d K,K1;
Eigen::Vector3d t_eye_in_hand(0.087639, -0.006377, 0.081998);
double tcp_x,tcp_y,tcp_z,offset_roll;
bool use_reciprocal_kdtree_,use_center_,use_sqpnp_;
char ini_path[] = "../config/charger.ini";
pcl::PointCloud<pcl::PointXYZ>::Ptr template_cloud(new pcl::PointCloud<pcl::PointXYZ>);
std::vector<Eigen::Vector3d> kps3d_circles_center;
std::vector<cv::Point2f> key_points2d_center;


void GolgInit(std::string &log_info_path)
{
    std::string log_info;
    DIR *dir;
    std::string str_command;
    if ((dir = opendir(log_info_path.c_str())) == NULL) {
        LOG(INFO) << "日志存放目录不存在，创建日志存放目录文件" << std::endl;
        str_command = "mkdir -p " + log_info_path;
        system(str_command.c_str()); // 存在就删除目录
    }

    log_info = log_info_path + "PL_Stereo_";
    FLAGS_alsologtostderr = true;
    FLAGS_colorlogtostderr = true;
    google::SetLogDestination(google::GLOG_INFO, log_info.c_str());
    google::SetLogFilenameExtension(".log");

    LOG(INFO) << "google::IsGoogleLoggingInitialized :" << google::IsGoogleLoggingInitialized() << std::endl;
    google::InitGoogleLogging("init logs------");
}

//ty
void eventCallback(TY_EVENT_INFO *event_info, void *userdata)
{
    if (event_info->eventId == TY_EVENT_DEVICE_OFFLINE)
    {
        LOG(INFO) << " === Event Callback: Device Offline!" << std::endl;
        // Note:
        //     Please set TY_BOOL_KEEP_ALIVE_ONOFF feature to false if you need to debug with breakpoint!
    }
    else if (event_info->eventId == TY_EVENT_LICENSE_ERROR)
    {
        LOG(INFO) << " === Event Callback: License Error!" << std::endl;
    }
}

//jaka
std::ostream& operator<<(std::ostream& output, std::vector<double> arr){
    for (uint i=0; i<arr.size(); ++i){
        output << arr[i] << ", ";
        output << "\n";
    }
    return output;
}


void sqpnp_data_trans(std::vector<sqpnp::_Point> &pts3D,std::vector<sqpnp::_Projection> &npts2D){
  kps3d_circles_center =
    {
        Eigen::Vector3d(0.0, 0.0, 0.0),
        Eigen::Vector3d(0.0, 0.034, 0.0),
        Eigen::Vector3d(-0.0198, 0.017, -0.00041),
        Eigen::Vector3d(-0.0198, 0.00275, 0.009596),
        Eigen::Vector3d(-0.0198, 0.03125, 0.009596),
        Eigen::Vector3d(0.0102, 0.017, 0.009596),
        Eigen::Vector3d(0.0222, 0.017, -0.00041),
        Eigen::Vector3d(0.0222, 0.005, 0.009596),
        Eigen::Vector3d(0.0222, 0.029, 0.009596),

        Eigen::Vector3d(0.0, -0.00615, 0.0),// 
        Eigen::Vector3d(0.00615, 0.0, 0.0),  //circle1 in
        Eigen::Vector3d(0.0, 0.00615, 0.0),   // 
        Eigen::Vector3d(-0.00615, 0.0, 0.0),   //
        // 
        Eigen::Vector3d(0.0, -0.0123, 0.0),     //
        Eigen::Vector3d(0.0123, 0.0, 0.0),     //
        Eigen::Vector3d(0.0, 0.0123, 0.0),    //circle1 out
        Eigen::Vector3d(-0.0123, 0.0, 0.0),  //
        //circle2in
        Eigen::Vector3d(0.0, 0.02785, 0.0),
        Eigen::Vector3d(0.00615, 0.034, 0.0),
        Eigen::Vector3d(0.0, 0.04015, 0.0),
        Eigen::Vector3d(-0.00615, 0.034, 0.0),
        //circle2out
        Eigen::Vector3d(0.0, 0.0217, 0.0),
        Eigen::Vector3d(0.0123, 0.034, 0.0),
        Eigen::Vector3d(0.0, 0.0463, 0.0),
        Eigen::Vector3d(-0.0123, 0.034, 0.0),
        //circle3
        Eigen::Vector3d(-0.0198, 0.00975, -0.00041),
        Eigen::Vector3d(-0.01255, 0.017, -0.00041),
        Eigen::Vector3d(-0.0198, 0.02425, -0.00041),
        Eigen::Vector3d(-0.02705, 0.017, -0.00041),
        //circle4
        Eigen::Vector3d(-0.0198, -0.00175, 0.009596),
        Eigen::Vector3d(-0.0153, 0.00275,   0.009596),
        Eigen::Vector3d(-0.0198, 0.00725,  0.009596),
        Eigen::Vector3d(-0.0243, 0.00275,  0.009596),
        //circle5
        Eigen::Vector3d(-0.0198, 0.02675, 0.009596),
        Eigen::Vector3d(-0.0153, 0.03125, 0.009596),
        Eigen::Vector3d(-0.0198, 0.03575, 0.009596),
        Eigen::Vector3d(-0.0243, 0.03125, 0.009596),
        //circle6
        Eigen::Vector3d(0.0102, 0.0125, 0.009596),
        Eigen::Vector3d(0.0147, 0.017,  0.009596),
        Eigen::Vector3d(0.0102, 0.0215, 0.009596),
        Eigen::Vector3d(0.0057, 0.017,  0.009596),
        //circle7
        Eigen::Vector3d(0.0222, 0.0125, -0.00041),
        Eigen::Vector3d(0.0267, 0.017, -0.00041),
        Eigen::Vector3d(0.0222, 0.0215, -0.00041),
        Eigen::Vector3d(0.0177, 0.017, -0.00041),
        //circle8
        Eigen::Vector3d(0.0222, 0.0005, 0.009596),
        Eigen::Vector3d(0.0267, 0.005, 0.009596),
        Eigen::Vector3d(0.0222, 0.0095, 0.009596),
        Eigen::Vector3d(0.0177, 0.005, 0.009596),
        // circle9
        Eigen::Vector3d(0.0222, 0.0245, 0.009596),
        Eigen::Vector3d(0.0267, 0.029, 0.009596),
        Eigen::Vector3d(0.0222, 0.0335, 0.009596),
        Eigen::Vector3d(0.0177, 0.029, 0.009596),
    };
    K<<
      1719.57,0, 953.387,
      0,1720.51, 546,
      0,0,1;
      K1=K.inverse();
  pts3D.reserve(pts3D.size());
  npts2D.reserve(key_points2d_center.size());
  for(size_t i =0 ; i<kps3d_circles_center.size();i++){
    pts3D.emplace_back(kps3d_circles_center[i]);
  }

  for(size_t i=0; i<key_points2d_center.size();i++){
    const double x=key_points2d_center[i].x, y=key_points2d_center[i].y;

    const double nx=K1(0, 0)*x + K1(0, 1)*y + K1(0, 2);
    const double ny=K1(1, 0)*x + K1(1, 1)*y + K1(1, 2);
    npts2D.emplace_back(nx, ny);
  }
}

// 检测框的标签
static const char *class_names[] = {
    "charger", "circle1", "circle2", "circle3", "circle4", "circle5","circle6","circle7","circle8",
    "circle9","circle10","circle11"};

// 字符串分割函数
std::vector<std::string> split(std::string str, std::string pattern)
{
  std::string::size_type pos;
  std::vector<std::string> result;
  str += pattern; // 扩展字符串以方便操作
  int size = str.size();
  for (int i = 0; i < size; i++)
  {
    pos = str.find(pattern, i);
    if (pos < size)
    {
      std::string s = str.substr(i, pos - i);
      result.push_back(s);
      i = pos + pattern.size() - 1;
    }
  }
  return result;
}

void cal_pos(cv::Mat &image,Eigen::Matrix<double,6,1> &result ,bool &use_center)
{
  LOG(INFO)<<"kps3d_circles_center size "<<kps3d_circles_center.size()<<std::endl;
  INIReader reader(ini_path);
  tcp_x=reader.GetReal("main_charger", "x", 0);
  tcp_y=reader.GetReal("main_charger", "y", 0);
  tcp_z=reader.GetReal("main_charger", "z", 0);
  offset_roll=reader.GetReal("main_charger", "roll", 0);
  use_reciprocal_kdtree_=reader.GetBoolean("main_charger", "use_reciprocal_kdtree", false);
  LOG(INFO) << " use_reciprocal_kdtree_ = "<<use_reciprocal_kdtree_<<std::endl;
  LOG(INFO) << " tcp_x = " << tcp_x << std::endl;
  LOG(INFO) << " tcp_y = " << tcp_y << std::endl;
  LOG(INFO) << " tcp_z = " << tcp_z << std::endl;
  LOG(INFO) << " offset_roll = " << offset_roll << std::endl;
  tcp<<tcp_x,tcp_y,tcp_z;

  pcl::io::loadPCDFile("../models/charger/GB_charger_selected_1_9.pcd", *template_cloud);
  Eigen::Matrix4f Rz_offset;
	Rz_offset<< cos(offset_roll), -sin(offset_roll), 0,0,
		sin(offset_roll), cos(offset_roll), 0,0,
		0, 0, 1,0,
		0, 0, 0,1;

	pcl::transformPointCloud(*template_cloud, *template_cloud, Rz_offset);
  pcl::io::savePCDFile("../models/charger/GB_charger_selected_1_9_offset.pcd", *template_cloud);
  double total_det_time = 0;
  double total_process_time = 0;
  R_eye_in_hand << 
  -0.027445, -0.999507 , -0.01527,
  0.99898 ,-0.026876, -0.036295,
  0.035867, -0.016251 , 0.999224;

  // load yolov8n model 
  auto pDet = new ahpc_detector();
  auto ret = pDet->init(ini_path); //
  if (ret != SDKErrorCode::OK)
  {
    LOG(INFO) << " init error: code =  " << ret << std::endl;
  }
  ret = pDet->load_model();
  if (ret != SDKErrorCode::OK)
  {
    LOG(INFO) << " load_model error: code =  " << ret << std::endl;
  }
  int img_format = 0;
  int boxes_size = 0;
  RectBox *pMyBoxes = nullptr;
  struct timeval t1, t2, t3;
  double timeuse;
  double timeuse_ms;
  int fontface = cv::FONT_HERSHEY_SIMPLEX;
  char *bname;
  cv::Mat image_copy=image.clone();
  if (image.empty())
  {
    LOG(INFO) << " input image is empty " << std::endl;
    return;
  }
  gettimeofday(&t1, NULL);
  ret = pDet->detect(image.data, img_format, image.cols, image.rows, &pMyBoxes, &boxes_size);
  LOG(INFO)<<"detect finish "<<std::endl;
  gettimeofday(&t2, NULL);
  timeuse = t2.tv_sec - t1.tv_sec + (t2.tv_usec - t1.tv_usec) / 1000000.0; //
  timeuse_ms = t2.tv_sec * 1000.0 - t1.tv_sec * 1000.0 + (t2.tv_usec - t1.tv_usec) / 1000.0;
  auto t_used = timeuse_ms; //
  total_det_time += t_used;
  LOG(INFO) << " detect time used is :" << t_used << " ms " << std::endl;
  LOG(INFO) << " boxes_size = " << boxes_size << std::endl;

  #ifdef SHOW_IMGS
  // roi,充电口区域
  cv::Mat output(image.rows, image.cols, CV_8UC1, cv::Scalar(0));
  cv::Rect2f mask_box(pMyBoxes[0].location[0], pMyBoxes[0].location[1], pMyBoxes[0].location[2], pMyBoxes[0].location[3]);

  cv::Mat roi = image(cv::Rect2f(pMyBoxes[0].location[0], pMyBoxes[0].location[1], pMyBoxes[0].location[2], pMyBoxes[0].location[3]));
  // std::string roi_name = out_path + currtime_ + "roi_name.jpg";
  // cv::imwrite(roi_name, roi);

  cv::Mat gray, blur, edges, hist, binary, opening, closing;
  cv::cvtColor(roi, gray, cv::COLOR_BGR2GRAY);

  // cv::GaussianBlur(gray, blur, cv::Size(5, 5), 0);
  cv::bilateralFilter(gray, blur, 5, 75, 75);

  cv::equalizeHist(blur, hist);
  cv::threshold(hist, binary, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

  // open
  cv::Mat kernel_1 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
  cv::morphologyEx(binary, opening, cv::MORPH_OPEN, kernel_1);

  // close
  cv::morphologyEx(opening, closing, cv::MORPH_CLOSE, kernel_1);

  // open
  cv::Mat kernel_2 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
  cv::morphologyEx(closing, opening, cv::MORPH_OPEN, kernel_2);

  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(opening, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);
  int maxlen = 0;
  int index = 0;
  //   LOG(INFO) << "contours.size(): " << contours.size() << std::endl;
  for (int i = 0; i < contours.size(); i++)
  {
    cv::Rect2f rect = cv::boundingRect(contours[i]);
    if (rect.x == 0 || rect.y == 0 || rect.width == roi.cols || rect.height == roi.rows)
      continue;
    int le = cv::arcLength(contours[i], true);
    if (le >= 100 & le > maxlen)
    {
      maxlen = le;
      index = i;
    }
  }

  cv::Mat maxcontour = cv::Mat::zeros(roi.rows, roi.cols, CV_8UC1);
  cv::drawContours(maxcontour, contours, index, 255, -1);
  cv::drawContours(maxcontour, contours, index, 0, 1);
  cv::Mat contoursbelowmax;
  cv::bitwise_and(opening, maxcontour, contoursbelowmax);

  key_points2d_center.resize(53);
  float c1_x, c1_y, r1, c2_x, c2_y, r2;

  for (int i = 1; i < boxes_size; i++)
  { 
    std::vector<cv::Point2f> circle_points(4);
    std::string label = class_names[pMyBoxes[i].label];
    // LOG(INFO) << "label: " << label << std::endl;
    float c_x = pMyBoxes[i].location[0] - pMyBoxes[0].location[0] + pMyBoxes[i].location[2] / 2;
    float c_y = pMyBoxes[i].location[1] - pMyBoxes[0].location[1] + pMyBoxes[i].location[3] / 2;
    float r = (pMyBoxes[i].location[2] + pMyBoxes[i].location[3]) / 4;
    // LOG(INFO) << " c_x = " << c_x << "c_y = " << c_y << "r = " << r << std::endl;
    float circle_left_x = pMyBoxes[i].location[0];
    float circle_left_y = pMyBoxes[i].location[1] + pMyBoxes[i].location[3] / 2;
    float circle_up_x = circle_left_x + pMyBoxes[i].location[2] / 2;
    float circle_up_y = pMyBoxes[i].location[1];
    float circle_right_x = circle_left_x + pMyBoxes[i].location[2];
    float circle_right_y = circle_left_y;
    float circle_down_x = circle_up_x;
    float circle_down_y = circle_up_y + pMyBoxes[i].location[3];
    float center_x =pMyBoxes[i].location[0]+pMyBoxes[i].location[2]/2;
    float center_y =pMyBoxes[i].location[1]+pMyBoxes[i].location[3]/2;
    circle_points[0] = cv::Point2f(circle_left_x, circle_left_y);
    circle_points[1] = cv::Point2f(circle_up_x, circle_up_y);
    circle_points[2] = cv::Point2f(circle_right_x, circle_right_y);
    circle_points[3] = cv::Point2f(circle_down_x, circle_down_y);
    
    if (label == "circle1")
    {
      c1_x = c_x;
      c1_y = c_y;
      r1 = r;
      key_points2d_center[0] = cv::Point2f(center_x,center_y);
      key_points2d_center[9] = circle_points[0];
      key_points2d_center[10] = circle_points[1];
      key_points2d_center[11] = circle_points[2];
      key_points2d_center[12] = circle_points[3];
    }
    else if (label == "circle2")
    {
      c2_x = c_x;
      c2_y = c_y;
      r2 = r;
      key_points2d_center[1] = cv::Point2f(center_x,center_y);
      key_points2d_center[17] = circle_points[0];
      key_points2d_center[18] = circle_points[1];
      key_points2d_center[19] = circle_points[2];
      key_points2d_center[20] = circle_points[3];
    }
    else if (label == "circle5")
    {
      key_points2d_center[2] = cv::Point2f(center_x,center_y);
      key_points2d_center[25] = circle_points[0];
      key_points2d_center[26] = circle_points[1];
      key_points2d_center[27] = circle_points[2];
      key_points2d_center[28] = circle_points[3];

    }
    else if (label == "circle6")
    {
      key_points2d_center[3] = cv::Point2f(center_x,center_y);
      key_points2d_center[29] = circle_points[0];
      key_points2d_center[30] = circle_points[1];
      key_points2d_center[31] = circle_points[2];
      key_points2d_center[32] = circle_points[3];
    }
    else if (label == "circle7")
    {
      key_points2d_center[4] = cv::Point2f(center_x,center_y);
      key_points2d_center[33] = circle_points[0];
      key_points2d_center[34] = circle_points[1];
      key_points2d_center[35] = circle_points[2];
      key_points2d_center[36] = circle_points[3];
      
    }
    else if (label == "circle8")
    {
      cv::circle(contoursbelowmax, cv::Point2f(c_x, c_y), r, cv::Scalar(255, 255, 255), -1);
      cv::circle(contoursbelowmax, cv::Point2f(c_x, c_y), r / 2, cv::Scalar(0, 0, 0), -1);
      key_points2d_center[5] = cv::Point2f(center_x,center_y);
      key_points2d_center[37] = circle_points[0];
      key_points2d_center[38] = circle_points[1];
      key_points2d_center[39] = circle_points[2];
      key_points2d_center[40] = circle_points[3];
    }
    else if (label == "circle9")
    {
      key_points2d_center[6] = cv::Point2f(center_x,center_y);
      key_points2d_center[41] = circle_points[0];
      key_points2d_center[42] = circle_points[1];
      key_points2d_center[43] = circle_points[2];
      key_points2d_center[44] = circle_points[3];
    }
    else if (label == "circle10")
    {

      key_points2d_center[7] = cv::Point2f(center_x,center_y);
      key_points2d_center[45] = circle_points[0];
      key_points2d_center[46] = circle_points[1];
      key_points2d_center[47] = circle_points[2];
      key_points2d_center[48] = circle_points[3];
    }
    else if (label == "circle11")
    {

      key_points2d_center[8] = cv::Point2f(center_x,center_y);
      key_points2d_center[49] = circle_points[0];
      key_points2d_center[50] = circle_points[1];
      key_points2d_center[51] = circle_points[2];
      key_points2d_center[52] = circle_points[3];
      
    }
    else if (label == "circle3")
    {
      key_points2d_center[13] = circle_points[0];
      key_points2d_center[14] = circle_points[1];
      key_points2d_center[15] = circle_points[2];
      key_points2d_center[16] = circle_points[3];
      cv::circle(contoursbelowmax, cv::Point2f(c_x, c_y), r, cv::Scalar(255, 255, 255), -1);
    }
    else if (label == "circle4")
    {
      key_points2d_center[1] = cv::Point2f(center_x,center_y);
      key_points2d_center[21] = circle_points[0];
      key_points2d_center[22] = circle_points[1];
      key_points2d_center[23] = circle_points[2];
      key_points2d_center[24] = circle_points[3];
      cv::circle(contoursbelowmax, cv::Point2f(c_x, c_y), r, cv::Scalar(255, 255, 255), -1);
    }
    else{
      LOG(INFO)<<"charger"<<std::endl;
    }

  }
  cv::circle(contoursbelowmax, cv::Point2f(c1_x, c1_y), r1, cv::Scalar(0, 0, 0), -1);
  cv::circle(contoursbelowmax, cv::Point2f(c2_x, c2_y), r2, cv::Scalar(0, 0, 0), -1);
  // std::string contoursbelowmax_name = out_path + currtime_ + "contoursbelowmax.jpg";
  // cv::imwrite(contoursbelowmax_name, contoursbelowmax);
  std::vector<cv::Point2f> model_points;
  for (int r = 0; r < contoursbelowmax.rows; r++)
  {
    for (int c = 0; c < contoursbelowmax.cols; c++)
    {
      if (contoursbelowmax.at<uchar>(r, c) == 255)
      {
        model_points.push_back(cv::Point2f(c + int(pMyBoxes[0].location[0]), r + int(pMyBoxes[0].location[1])));
        // cv::circle(image, cv::Point2f(c + int(pMyBoxes[0].location[0]), r + int(pMyBoxes[0].location[1])), 1, cv::Scalar(0, 0, 255), 1);
      }
    }
  }
  LOG(INFO) << "model_points.size() = " << model_points.size() << std::endl;
  LOG(INFO) << "key_points2d_center.size() = " << key_points2d_center.size() << std::endl;

  contoursbelowmax.copyTo(output(mask_box));
  // cv::imshow("output", output);
  // cv::waitKey();
  //  std::string output_name = out_path + currtime_ + "output.jpg";
  // cv::imwrite(output_name, output);
  cv::Mat output_color, labels, stats, centroids;
  int num_labels = cv::connectedComponentsWithStats(output, labels, stats, centroids, 8);
  std::vector<cv::Vec3b> color(num_labels + 1);
  color[0] = cv::Vec3b(0, 0, 0);
  for (int i = 1; i <= num_labels; i++)
  {
    color[i] = cv::Vec3b(std::rand() % 256, std::rand() % 256, std::rand() % 256);
    if (stats.at<int>(i - 1, cv::CC_STAT_AREA) < 50)
    {
      color[i] = cv::Vec3b(0, 0, 0);
      continue;
    }
  }
  output_color = cv::Mat::zeros(output.size(), CV_8UC3);
  for (int r = 0; r < output_color.rows; r++)
  { // 遍历每个像素点，根据其标签上色
    for (int c = 0; c < output_color.cols; c++)
    {
      int label = labels.at<int>(r, c);
      output_color.at<cv::Vec3b>(r, c) = color[label];
    }
  }
  // std::string output_color_name = out_path + currtime_ + "output_color.jpg";
  // cv::imwrite(output_color_name, output_color);


  //检测框可视化
  for (int i = 0; i < boxes_size; i++)
  {
    std::string label = class_names[pMyBoxes[i].label];
    std::string text = label + ":" + std::to_string(pMyBoxes[i].score);
    auto det_result = pMyBoxes[i];
    float x1 = det_result.location[0];
    float y1 = det_result.location[1];
    float x2 = det_result.location[2] + det_result.location[0]; // w + x0
    float y2 = det_result.location[3] + det_result.location[1]; // h + y0
    cv::Point2f l_d(x1, y2);
    cv::Rect2f box(x1, y1, x2 - x1, y2 - y1);
    cv::rectangle(image, box, cv::Scalar(0, 255, 0), 1, 8, 0);
    std::vector<std::string> str = split(text, "/n");
    // for (int j = 0; j < str.size(); j++)
    // {
    //   int y = l_d.y + (j + 1) * 15;
    //   cv::Point ld(l_d.x, y);
    //   cv::putText(image, str[j], ld, fontface, 0.5, cv::Scalar(255, 0, 0), 1, 8);
    // }
  }
  gettimeofday(&t3, NULL);
  timeuse = t3.tv_sec - t2.tv_sec + (t3.tv_usec - t2.tv_usec) / 1000000.0; //
  timeuse_ms = t3.tv_sec * 1000.0 - t2.tv_sec * 1000.0 + (t3.tv_usec - t2.tv_usec) / 1000.0;
  t_used = timeuse_ms; //
  total_process_time += t_used;
  LOG(INFO) << " process time used is :" << t_used << " ms " << std::endl;

  std::string out_name = out_path + currtime_ + "roi.jpg";
  cv::imwrite(out_name, image);

  std::shared_ptr<mvgpcgen::camera::Perspective> camera_ptr = std::make_shared<mvgpcgen::camera::Perspective>(
      cam_K[0], cam_K[1], cam_K[2], cam_K[3], cam_K[4], cam_K[5], cam_K[6], cam_K[7]);
  std::shared_ptr<mvgpcgen::solve::BlindPnp> blind_pnp_ptr = std::make_shared<mvgpcgen::solve::BlindPnp>(
      "../models/charger/GB_charger_selected_1_9_offset.pcd");

  
  blind_pnp_ptr->setCameraModel(camera_ptr);

  // 计算初始位姿
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
  Eigen::Vector3d t = Eigen::Vector3d::Zero();
  
  robust_pose_pnp::Matrix34d bestRt;
  std::vector<int> outidx;
  std::vector<sqpnp::_Point> pts3D;
  std::vector<sqpnp::_Projection> npts2D;
  sqpnp_data_trans(pts3D,npts2D);
  if(!use_sqpnp_){
    LOG(INFO)<<"correspondingPnpSolve";
  blind_pnp_ptr->correspondingPnpSolve(key_points2d_center, R, t);
  LOG(INFO) << "First optim res:\n R=\n"
            << R << "\nt=" << t << "\n";
  }
  else{

  robust_pose_pnp::PoseEstimator rpe(&pts3D, &npts2D, 4 /*3*/, 50);
  //rpe.set_sample_sizes(4, 30); // changes sample sizes dynamically
  rpe.ransacfit(25, 200, 0.8, 0.01, bestRt, nullptr, &outidx);

  LOG(INFO) << "\nRANSAC estimate\n"<<bestRt << std::endl;
  R<<bestRt(0,0),bestRt(0,1),bestRt(0,2),
    bestRt(1,0),bestRt(1,1),bestRt(1,2),
    bestRt(2,0),bestRt(2,1),bestRt(2,2);
  t<<bestRt(0,3),bestRt(1,3),bestRt(2,3);

  }

  //3d特征点投影到2d图
  Eigen::Matrix4d trans_matrix;
   ResultProcess::GetTransformationMatrix(R,t,trans_matrix);
  pcl::PointCloud<pcl::PointXYZ>::Ptr center_trans_cloud(new pcl::PointCloud<pcl::PointXYZ>);

  Eigen::Vector2d center_trans_uv;
  for(size_t i=0;i<kps3d_circles_center.size();i++){
    pcl::PointXYZ point;
    point.x=kps3d_circles_center[i][0];
    point.y=kps3d_circles_center[i][1];
    point.z=kps3d_circles_center[i][2];
    center_trans_cloud->push_back(point);
    
  }
  	pcl::transformPointCloud(*center_trans_cloud, *center_trans_cloud, trans_matrix.cast<float>());

  for(size_t i=0 ;i<center_trans_cloud->size();i++){
    center_trans_uv[0]=1719.57*center_trans_cloud->points[i].x/center_trans_cloud->points[i].z +953.387;
    center_trans_uv[1]=1720.51*center_trans_cloud->points[i].y/center_trans_cloud->points[i].z +546;
    //保存至原图中
    cv::circle(image, cv::Point2d(center_trans_uv[0],center_trans_uv[1]), 1, cv::Scalar(0, 255, 0), -1);
  }
  // 输入分割后的点，输出第二次优化后的位姿
  // blind_pnp_ptr->unknownCorrespondingPnpSolve(model_points, R, t,use_reciprocal_kdtree_);
  // LOG(INFO) << "Final optim res:\n R=\n"
  //           << R << "\nt=" << t << "\n";

  pcl::PointCloud<pcl::PointXYZ>::Ptr trans_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  
  Eigen::Vector2d trans_uv;
  std::vector<Eigen::Vector2d> V_trans_uv;
	pcl::transformPointCloud(*template_cloud, *trans_cloud, trans_matrix.cast<float>());
  cv::Mat test_gray;
  cv::cvtColor(image, test_gray, cv::COLOR_BGR2GRAY);

  for(int i=0;i<1080;i++){
    for(int j=0;j<1920;j++){
      test_gray.at<uchar>(i,j)=137;
    }
  }
  for(size_t i=0;i<trans_cloud->size();i++){
    trans_uv[0]=1719.57*trans_cloud->points[i].x/trans_cloud->points[i].z +953.387;
    trans_uv[1]=1720.51*trans_cloud->points[i].y/trans_cloud->points[i].z +546;
    //保存投影至纯灰图内
    cv::circle(test_gray, cv::Point2d(trans_uv[0],trans_uv[1]), 1, cv::Scalar(0, 0, 0), -1);
    //保存至原图中
    // cv::circle(image, cv::Point2d(trans_uv[0],trans_uv[1]), 1, cv::Scalar(0, 0, 0), -1);

    // V_trans_uv.push_back(trans_uv);
  }


  std::string projection_out_name = out_path + currtime_ + "projection.jpg";
  cv::imwrite(projection_out_name, image);

  ResultProcess::GetFinalResult(R, t,R_eye_in_hand,t_eye_in_hand,tcp,result);

  Eigen::Vector3d pt3d = blind_pnp_ptr->cad_3d_.at(0);

  // 显示投影结果
  for (uint i = 0; i < blind_pnp_ptr->cad_3d_.size(); ++i)
  {
    Eigen::Vector3d pt3d = blind_pnp_ptr->cad_3d_.at(i);
    Eigen::Vector3d pt_cam = R * pt3d + t;

    double u, v;
    camera_ptr->cameraToPixel(pt_cam.x(), pt_cam.y(), pt_cam.z(), u, v);
    cv::circle(image, cv::Point2d(u, v), 1, cv::Scalar(255, 123, 0), -1);
  }
  
  // std::string projection_out_name = out_path + currtime_ + "projection.jpg";
  // cv::imwrite(projection_out_name, image);
  // cv::imshow("projection result", image);
  // cv::waitKey();

  camera_ptr.reset();

  if (camera_ptr == nullptr)
  {
    LOG(INFO) << "camera_ptr has been reset." << std::endl;
  }

  blind_pnp_ptr.reset();

  if (blind_pnp_ptr == nullptr)
  {
    LOG(INFO) << "blind_pnp_ptr has been reset." << std::endl;
  }

#endif

  pDet->release(); 
  if (pDet)
  {
    delete pDet;
    pDet = NULL;
  }


}

void cal_pos(cv::Mat& image,Eigen::Matrix<double, 6, 1>& result)
{ 
 
  INIReader reader(ini_path);
  tcp_x=reader.GetReal("main_charger", "x", 0);
  tcp_y=reader.GetReal("main_charger", "y", 0);
  tcp_z=reader.GetReal("main_charger", "z", 0);
  offset_roll=reader.GetReal("main_charger", "roll", 0);
  use_reciprocal_kdtree_=reader.GetBoolean("main_charger", "use_reciprocal_kdtree", false);
  LOG(INFO) << " use_reciprocal_kdtree_ = "<<use_reciprocal_kdtree_<<std::endl;
  LOG(INFO) << " tcp_x = " << tcp_x << std::endl;
  LOG(INFO) << " tcp_y = " << tcp_y << std::endl;
  LOG(INFO) << " tcp_z = " << tcp_z << std::endl;
  LOG(INFO) << " offset_roll = " << offset_roll << std::endl;
  tcp<<tcp_x,tcp_y,tcp_z;

  pcl::io::loadPCDFile("../models/charger/eletre_selected_offset.pcd", *template_cloud);
  Eigen::Matrix4f Rz_offset;
	Rz_offset<< cos(offset_roll), -sin(offset_roll), 0,0,
		sin(offset_roll), cos(offset_roll), 0,0,
		0, 0, 1,0,
		0, 0, 0,1;

	pcl::transformPointCloud(*template_cloud, *template_cloud, Rz_offset);
  pcl::io::savePCDFile("../models/charger/eletre_selected_offset.pcd", *template_cloud);
 


  double total_det_time = 0;
  double total_process_time = 0;
  R_eye_in_hand << -0.027445, -0.999507 , -0.01527,
  0.99898 ,-0.026876, -0.036295,
 0.035867, -0.016251 , 0.999224;

  // load yolov8n model
  auto pDet = new ahpc_detector();
  auto ret = pDet->init(ini_path); //
  if (ret != SDKErrorCode::OK)
  {
    LOG(INFO) << " init error: code =  " << ret << std::endl;
  }
  ret = pDet->load_model();
  if (ret != SDKErrorCode::OK)
  {
    LOG(INFO) << " load_model error: code =  " << ret << std::endl;
  }
  int img_format = 0;
  int boxes_size = 0;

  RectBox *pMyBoxes = nullptr;
  struct timeval t1, t2, t3;
  double timeuse;
  double timeuse_ms;
  int fontface = cv::FONT_HERSHEY_SIMPLEX;
  char *bname;
  if (image.empty())
  {
    LOG(INFO) << " input image is empty " << std::endl;
    return;
  }
  gettimeofday(&t1, NULL);
  ret = pDet->detect(image.data, img_format, image.cols, image.rows, &pMyBoxes, &boxes_size);
  gettimeofday(&t2, NULL);
  timeuse = t2.tv_sec - t1.tv_sec + (t2.tv_usec - t1.tv_usec) / 1000000.0; //
  timeuse_ms = t2.tv_sec * 1000.0 - t1.tv_sec * 1000.0 + (t2.tv_usec - t1.tv_usec) / 1000.0;
  auto t_used = timeuse_ms; //
  total_det_time += t_used;
  LOG(INFO) << " detect time used is :" << t_used << " ms " << std::endl;
  LOG(INFO) << " boxes_size = " << boxes_size << std::endl;

#ifdef SHOW_IMGS
  // roi
  cv::Mat output(image.rows, image.cols, CV_8UC1, cv::Scalar(0));
  cv::Rect2f mask_box(pMyBoxes[0].location[0], pMyBoxes[0].location[1], pMyBoxes[0].location[2], pMyBoxes[0].location[3]);

  cv::Mat roi = image(cv::Rect2f(pMyBoxes[0].location[0], pMyBoxes[0].location[1], pMyBoxes[0].location[2], pMyBoxes[0].location[3]));
  // std::string roi_name = out_path + currtime_ + "roi_name.jpg";
  // cv::imwrite(roi_name, roi);

  cv::Mat gray, blur, edges, hist, binary, opening, closing;
  cv::cvtColor(roi, gray, cv::COLOR_BGR2GRAY);

  // cv::GaussianBlur(gray, blur, cv::Size(5, 5), 0);
  cv::bilateralFilter(gray, blur, 5, 75, 75);

  cv::equalizeHist(blur, hist);
  cv::threshold(hist, binary, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

  // open
  cv::Mat kernel_1 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
  cv::morphologyEx(binary, opening, cv::MORPH_OPEN, kernel_1);

  // close
  cv::morphologyEx(opening, closing, cv::MORPH_CLOSE, kernel_1);

  // open
  cv::Mat kernel_2 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
  cv::morphologyEx(closing, opening, cv::MORPH_OPEN, kernel_2);

  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(opening, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);
  int maxlen = 0;
  int index = 0;
  //   LOG(INFO) << "contours.size(): " << contours.size() << std::endl;
  for (int i = 0; i < contours.size(); i++)
  {
    cv::Rect2f rect = cv::boundingRect(contours[i]);
    if (rect.x == 0 || rect.y == 0 || rect.width == roi.cols || rect.height == roi.rows)
      continue;
    int le = cv::arcLength(contours[i], true);
    if (le >= 100 & le > maxlen)
    {
      maxlen = le;
      index = i;
    }
  }

  cv::Mat maxcontour = cv::Mat::zeros(roi.rows, roi.cols, CV_8UC1);
  cv::drawContours(maxcontour, contours, index, 255, -1);
  cv::drawContours(maxcontour, contours, index, 0, 1);
  cv::Mat contoursbelowmax;
  cv::bitwise_and(opening, maxcontour, contoursbelowmax);

  std::vector<std::vector<cv::Point2f>> key_points(5);
  float c1_x, c1_y, r1, c2_x, c2_y, r2;

  for (int i = 1; i < boxes_size; i++)
  {
    std::vector<cv::Point2f> circle_points(4);
    std::string label = class_names[pMyBoxes[i].label];
    // LOG(INFO) << "label: " << label << std::endl;
    float c_x = pMyBoxes[i].location[0] - pMyBoxes[0].location[0] + pMyBoxes[i].location[2] / 2;
    float c_y = pMyBoxes[i].location[1] - pMyBoxes[0].location[1] + pMyBoxes[i].location[3] / 2;
    float r = (pMyBoxes[i].location[2] + pMyBoxes[i].location[3]) / 4;
    // LOG(INFO) << " c_x = " << c_x << "c_y = " << c_y << "r = " << r << std::endl;
    float circle_left_x = pMyBoxes[i].location[0];
    float circle_left_y = pMyBoxes[i].location[1] + pMyBoxes[i].location[3] / 2;
    float circle_up_x = circle_left_x + pMyBoxes[i].location[2] / 2;
    float circle_up_y = pMyBoxes[i].location[1];
    float circle_right_x = circle_left_x + pMyBoxes[i].location[2];
    float circle_right_y = circle_left_y;
    float circle_down_x = circle_up_x;
    float circle_down_y = circle_up_y + pMyBoxes[i].location[3];
    circle_points[0] = cv::Point2f(circle_left_x, circle_left_y);
    circle_points[1] = cv::Point2f(circle_up_x, circle_up_y);
    circle_points[2] = cv::Point2f(circle_right_x, circle_right_y);
    circle_points[3] = cv::Point2f(circle_down_x, circle_down_y);

    if (label == "circle1")
    {
      c1_x = c_x;
      c1_y = c_y;
      r1 = r;
      key_points[0] = circle_points;
    }
    else if (label == "circle2")
    {
      c2_x = c_x;
      c2_y = c_y;
      r2 = r;
      key_points[2] = circle_points;
    }
    else if (label == "circle3")
    {
      cv::circle(contoursbelowmax, cv::Point2f(c_x, c_y), r, cv::Scalar(255, 255, 255), -1);
      key_points[1] = circle_points;
    }
    else if (label == "circle4")
    {
      cv::circle(contoursbelowmax, cv::Point2f(c_x, c_y), r, cv::Scalar(255, 255, 255), -1);
      key_points[3] = circle_points;
    }
    else if (label == "circle5")
    {
      cv::circle(contoursbelowmax, cv::Point2f(c_x, c_y), r, cv::Scalar(255, 255, 255), -1);
      cv::circle(contoursbelowmax, cv::Point2f(c_x, c_y), r / 2, cv::Scalar(0, 0, 0), -1);
      key_points[4] = circle_points;
    }
  }
  cv::circle(contoursbelowmax, cv::Point2f(c1_x, c1_y), r1, cv::Scalar(0, 0, 0), -1);
  cv::circle(contoursbelowmax, cv::Point2f(c2_x, c2_y), r2, cv::Scalar(0, 0, 0), -1);
  std::string contoursbelowmax_name = out_path + currtime_ + "contoursbelowmax.jpg";
  cv::imwrite(contoursbelowmax_name, contoursbelowmax);

  std::vector<cv::Point2f> model_points;
  for (int r = 0; r < contoursbelowmax.rows; r++)
  {
    for (int c = 0; c < contoursbelowmax.cols; c++)
    {
      if (contoursbelowmax.at<uchar>(r, c) == 255)
      {
        model_points.push_back(cv::Point2f(c + int(pMyBoxes[0].location[0]), r + int(pMyBoxes[0].location[1])));
        cv::circle(image, cv::Point2f(c + int(pMyBoxes[0].location[0]), r + int(pMyBoxes[0].location[1])), 1, cv::Scalar(0, 0, 255), 1);
      }
    }
  }
  LOG(INFO) << "model_points.size() = " << model_points.size() << std::endl;

  contoursbelowmax.copyTo(output(mask_box));
  // cv::imshow("output", output);
  // cv::waitKey();
  //  std::string output_name = out_path + currtime_ + "output.jpg";
  // cv::imwrite(output_name, output);
  cv::Mat output_color, labels, stats, centroids;
  int num_labels = cv::connectedComponentsWithStats(output, labels, stats, centroids, 8);
  std::vector<cv::Vec3b> color(num_labels + 1);
  color[0] = cv::Vec3b(0, 0, 0);
  for (int i = 1; i <= num_labels; i++)
  {
    color[i] = cv::Vec3b(std::rand() % 256, std::rand() % 256, std::rand() % 256);
    if (stats.at<int>(i - 1, cv::CC_STAT_AREA) < 50)
    {
      color[i] = cv::Vec3b(0, 0, 0);
      continue;
    }
  }
  output_color = cv::Mat::zeros(output.size(), CV_8UC3);
  for (int r = 0; r < output_color.rows; r++)
  { // 遍历每个像素点，根据其标签上色
    for (int c = 0; c < output_color.cols; c++)
    {
      int label = labels.at<int>(r, c);
      output_color.at<cv::Vec3b>(r, c) = color[label];
    }
  }
  // std::string output_color_name = out_path + currtime_ + "output_color.jpg";
  // cv::imwrite(output_color_name, output_color);


  //检测框可视化
  for (int i = 0; i < boxes_size; i++)
  {
    std::string label = class_names[pMyBoxes[i].label];
    std::string text = label + ":" + std::to_string(pMyBoxes[i].score);
    auto det_result = pMyBoxes[i];
    float x1 = det_result.location[0];
    float y1 = det_result.location[1];
    float x2 = det_result.location[2] + det_result.location[0]; // w + x0
    float y2 = det_result.location[3] + det_result.location[1]; // h + y0
    cv::Point2f l_d(x1, y2);
    cv::Rect2f box(x1, y1, x2 - x1, y2 - y1);
    cv::rectangle(image, box, cv::Scalar(0, 255, 0), 1, 8, 0);
    std::vector<std::string> str = split(text, "/n");
    for (int j = 0; j < str.size(); j++)
    {
      int y = l_d.y + (j + 1) * 15;
      cv::Point ld(l_d.x, y);
      cv::putText(image, str[j], ld, fontface, 0.5, cv::Scalar(255, 0, 0), 1, 8);
    }
  }
  gettimeofday(&t3, NULL);
  timeuse = t3.tv_sec - t2.tv_sec + (t3.tv_usec - t2.tv_usec) / 1000000.0; //
  timeuse_ms = t3.tv_sec * 1000.0 - t2.tv_sec * 1000.0 + (t3.tv_usec - t2.tv_usec) / 1000.0;
  t_used = timeuse_ms; //
  total_process_time += t_used;
  LOG(INFO) << " process time used is :" << t_used << " ms " << std::endl;

  std::string out_name = out_path + currtime_ + "roi.jpg";
  cv::imwrite(out_name, image);

  std::shared_ptr<mvgpcgen::camera::Perspective> camera_ptr = std::make_shared<mvgpcgen::camera::Perspective>(
      cam_K[0], cam_K[1], cam_K[2], cam_K[3], cam_K[4], cam_K[5], cam_K[6], cam_K[7]);
  std::shared_ptr<mvgpcgen::solve::BlindPnp> blind_pnp_ptr = std::make_shared<mvgpcgen::solve::BlindPnp>(
      "../models/charger/eletre_selected_offset.pcd");
  blind_pnp_ptr->setCameraModel(camera_ptr);

  // 计算初始位姿
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
  Eigen::Vector3d t = Eigen::Vector3d::Zero();
  blind_pnp_ptr->correspondingPnpSolve(key_points, R, t);
  LOG(INFO) << "First optim res:\n R=\n"
            << R << "\nt=" << t << "\n";

  // 输入分割后的点，输出第二次优化后的位姿
  blind_pnp_ptr->unknownCorrespondingPnpSolve(model_points, R, t,use_reciprocal_kdtree_);
  LOG(INFO) << "Final optim res:\n R=\n"
            << R << "\nt=" << t << "\n";

  pcl::PointCloud<pcl::PointXYZ>::Ptr trans_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  
  Eigen::Matrix4d trans_matrix;
   ResultProcess::GetTransformationMatrix(R,t,trans_matrix);
  Eigen::Vector2d trans_uv;
  std::vector<Eigen::Vector2d> V_trans_uv;
	pcl::transformPointCloud(*template_cloud, *trans_cloud, trans_matrix.cast<float>());
  cv::Mat test_gray;
  cv::cvtColor(image, test_gray, cv::COLOR_BGR2GRAY);

  for(int i=0;i<1080;i++){
    for(int j=0;j<1920;j++){
      test_gray.at<uchar>(i,j)=137;
    }
  }
  for(size_t i=0;i<trans_cloud->size();i++){
    trans_uv[0]=1719.57*trans_cloud->points[i].x/trans_cloud->points[i].z +953.387;
    trans_uv[1]=1720.51*trans_cloud->points[i].y/trans_cloud->points[i].z +546;
    //保存投影至纯灰图内
    cv::circle(test_gray, cv::Point2d(trans_uv[0],trans_uv[1]), 1, cv::Scalar(0, 0, 0), -1);
    //保存至原图中
    cv::circle(image, cv::Point2d(trans_uv[0],trans_uv[1]), 1, cv::Scalar(0, 0, 0), -1);

    // V_trans_uv.push_back(trans_uv);
  }


  std::string projection_out_name = out_path + currtime_ + "projection.jpg";
  cv::imwrite(projection_out_name, image);

  ResultProcess::GetFinalResult(R, t,R_eye_in_hand,t_eye_in_hand,tcp,result);

  Eigen::Vector3d pt3d = blind_pnp_ptr->cad_3d_.at(0);

  // 显示投影结果
  for (uint i = 0; i < blind_pnp_ptr->cad_3d_.size(); ++i)
  {
    Eigen::Vector3d pt3d = blind_pnp_ptr->cad_3d_.at(i);
    Eigen::Vector3d pt_cam = R * pt3d + t;

    double u, v;
    camera_ptr->cameraToPixel(pt_cam.x(), pt_cam.y(), pt_cam.z(), u, v);
    cv::circle(image, cv::Point2d(u, v), 1, cv::Scalar(255, 123, 0), -1);
  }
  
  // std::string projection_out_name = out_path + currtime_ + "projection.jpg";
  // cv::imwrite(projection_out_name, image);
  // cv::imshow("projection result", image);
  // cv::waitKey();

  camera_ptr.reset();

  if (camera_ptr == nullptr)
  {
    LOG(INFO) << "camera_ptr has been reset." << std::endl;
  }

  blind_pnp_ptr.reset();

  if (blind_pnp_ptr == nullptr)
  {
    LOG(INFO) << "blind_pnp_ptr has been reset." << std::endl;
  }

#endif

  pDet->release(); 
  if (pDet)
  {
    delete pDet;
    pDet = NULL;
  }

}

void get_color(cv::Mat& color) 
{
//get ty color
  std::string ID, IP;
  TY_INTERFACE_HANDLE hIface = NULL;
  TY_ISP_HANDLE hColorIspHandle = NULL;
  TY_DEV_HANDLE hDevice = NULL;
  cv::Mat depth, irl, irr;

  LOGD("=== Init lib");
  ASSERT_OK(TYInitLib());
  TY_VERSION_INFO ver;
  ASSERT_OK(TYLibVersion(&ver));
  LOGD("     - lib version: %d.%d.%d", ver.major, ver.minor, ver.patch);

  std::vector<TY_DEVICE_BASE_INFO> selected;
  ASSERT_OK(selectDevice(TY_INTERFACE_ALL, ID, IP, 1, selected));
  ASSERT(selected.size() > 0);
  TY_DEVICE_BASE_INFO &selectedDev = selected[0];

  ASSERT_OK(TYOpenInterface(selectedDev.iface.id, &hIface));
  ASSERT_OK(TYOpenDevice(hIface, selectedDev.id, &hDevice));

  int32_t allComps;
  ASSERT_OK( TYGetComponentIDs(hDevice, &allComps) );

  ///try to enable color camera
  if(allComps & TY_COMPONENT_RGB_CAM) {
      LOGD("Has RGB camera, open RGB cam");
      ASSERT_OK( TYEnableComponents(hDevice, TY_COMPONENT_RGB_CAM) );
      //create a isp handle to convert raw image(color bayer format) to rgb image
      ASSERT_OK(TYISPCreate(&hColorIspHandle));
      //Init code can be modified in common.hpp
      //NOTE: Should set RGB image format & size before init ISP
      ASSERT_OK(ColorIspInitSetting(hColorIspHandle, hDevice));
      //You can  call follow function to show  color isp supported features
#if 0
      ColorIspShowSupportedFeatures(hColorIspHandle);
#endif
      //You can turn on auto exposure function as follow ,but frame rate may reduce .
      //Device may be casually stucked  1~2 seconds while software is trying to adjust device exposure time value
#if 0
      ASSERT_OK(ColorIspInitAutoExposure(hColorIspHandle, hDevice));
#endif
  }

  if (allComps & TY_COMPONENT_IR_CAM_LEFT) {
      LOGD("Has IR left camera, open IR left cam");
      ASSERT_OK(TYEnableComponents(hDevice, TY_COMPONENT_IR_CAM_LEFT));
  }

  if (allComps & TY_COMPONENT_IR_CAM_RIGHT) {
      LOGD("Has IR right camera, open IR right cam");
      ASSERT_OK(TYEnableComponents(hDevice, TY_COMPONENT_IR_CAM_RIGHT));
  }

  //try to enable depth map
  LOGD("Configure components, open depth cam");
  DepthViewer depthViewer("Depth");
  if (allComps & TY_COMPONENT_DEPTH_CAM) {
      int32_t image_mode;
      ASSERT_OK(get_default_image_mode(hDevice, TY_COMPONENT_DEPTH_CAM, image_mode));
      LOGD("Select Depth Image Mode: %dx%d", TYImageWidth(image_mode), TYImageHeight(image_mode));
      ASSERT_OK(TYSetEnum(hDevice, TY_COMPONENT_DEPTH_CAM, TY_ENUM_IMAGE_MODE, image_mode));
      ASSERT_OK(TYEnableComponents(hDevice, TY_COMPONENT_DEPTH_CAM));

      //depth map pixel format is uint16_t ,which default unit is  1 mm
      //the acutal depth (mm)= PixelValue * ScaleUnit 
      float scale_unit = 1.;
      TYGetFloat(hDevice, TY_COMPONENT_DEPTH_CAM, TY_FLOAT_SCALE_UNIT, &scale_unit);
      depthViewer.depth_scale_unit = scale_unit;
  }

  LOGD("Prepare image buffer");
  uint32_t frameSize;
  ASSERT_OK( TYGetFrameBufferSize(hDevice, &frameSize) );
  LOGD("     - Get size of framebuffer, %d", frameSize);

  LOGD("     - Allocate & enqueue buffers");
  char* frameBuffer[2];
  frameBuffer[0] = new char[frameSize];
  frameBuffer[1] = new char[frameSize];
  LOGD("     - Enqueue buffer (%p, %d)", frameBuffer[0], frameSize);
  ASSERT_OK( TYEnqueueBuffer(hDevice, frameBuffer[0], frameSize) );
  LOGD("     - Enqueue buffer (%p, %d)", frameBuffer[1], frameSize);
  ASSERT_OK( TYEnqueueBuffer(hDevice, frameBuffer[1], frameSize) );

  LOGD("Register event callback");
  ASSERT_OK(TYRegisterEventCallback(hDevice, eventCallback, NULL));

  bool hasTrigger;
  ASSERT_OK(TYHasFeature(hDevice, TY_COMPONENT_DEVICE, TY_STRUCT_TRIGGER_PARAM, &hasTrigger));
  if (hasTrigger) {
      LOGD("Disable trigger mode");
      TY_TRIGGER_PARAM trigger;
      trigger.mode = TY_TRIGGER_MODE_OFF;
      ASSERT_OK(TYSetStruct(hDevice, TY_COMPONENT_DEVICE, TY_STRUCT_TRIGGER_PARAM, &trigger, sizeof(trigger)));
  }

  LOGD("Start capture");
  ASSERT_OK( TYStartCapture(hDevice) );

  LOGD("While loop to fetch frame");
  bool exit_main = false;
  TY_FRAME_DATA frame;
  int index = 0;
  while(!exit_main) {
      int err = TYFetchFrame(hDevice, &frame, -1);
      if( err == TY_STATUS_OK ) {
          LOGD("Get frame %d", ++index);

          int fps = get_fps();
          if (fps > 0){
              LOGI("fps: %d", fps);
          }
          parseFrame(frame, &depth, &irl, &irr, &color, hColorIspHandle);
          // if(!depth.empty()){
          //     depthViewer.show(depth);
          // }
          // if(!irl.empty()){ cv::imshow("LeftIR", irl); }
          // if(!irr.empty()){ cv::imshow("RightIR", irr); }
          if(!color.empty()){ exit_main=true;; }

          int key = cv::waitKey(1);
          switch(key & 0xff) {
          case 0xff:
              break;
          case 'q':
              exit_main = true;
              break;
          default:
              LOGD("Unmapped key %d", key);
          }

          TYISPUpdateDevice(hColorIspHandle);
          LOGD("Re-enqueue buffer(%p, %d)"
              , frame.userBuffer, frame.bufferSize);
          ASSERT_OK( TYEnqueueBuffer(hDevice, frame.userBuffer, frame.bufferSize) );
      }
    
  }
  ASSERT_OK( TYStopCapture(hDevice) );
  ASSERT_OK( TYCloseDevice(hDevice));
  ASSERT_OK( TYCloseInterface(hIface) );
  ASSERT_OK(TYISPRelease(&hColorIspHandle));
  ASSERT_OK( TYDeinitLib() );
  delete frameBuffer[0];
  delete frameBuffer[1];

  LOGD("Main done!");
}

int main(int argc, char **argv)
{
  std::string log_info_path = "../data/logs/main_charger/";
  GolgInit(log_info_path);
  cv::Mat image = cv::imread("../data/images/test/eletre_test.jpg");
  INIReader reader(ini_path);
  use_center_=reader.GetBoolean("main_charger", "use_center", false);
  use_sqpnp_=reader.GetBoolean("main_charger", "use_sqpnp", false);
  // cv::Mat image;
  
  out_path = "../data/images/out_path/";
  Eigen::Matrix<double, 6, 1> result;
  // get_color(image);
  time_t a = time(NULL);
  tm currtime = *localtime(&a);
  currtime_ = std::to_string(currtime.tm_year + 1900) + std::to_string(currtime.tm_mon + 1) +
                          std::to_string(currtime.tm_mday) + "_" + std::to_string(currtime.tm_hour) + "_" + std::to_string(currtime.tm_min) +
                          "_" + std::to_string(currtime.tm_sec);
  std::string out_name = out_path + currtime_ + ".jpg";
  LOG(INFO)<<"out name ："<<out_name<<std::endl;
  cv::imwrite(out_name, image);

  if(use_center_)
  {
    cal_pos(image,result,use_center_);
  }
  else
  {
    cal_pos(image,result);
  }

  // linear move

  JakaMoveInterface::Ptr_ jaka_move_interface = JakaMoveInterface::Ptr_(new JakaMoveInterface("169.254.232.101"));
  jaka_move_interface->init();
  jaka_move_interface->setPayload(3.8,-80.64,56.1,57.268);//with gun
  char name[50] = "connector";
  jaka_move_interface->setTcp(1,name,-25.041, -19.909, 327.973);
  
  std::vector<double> pose_tcp_target = {result(0), result(1), result(2), result(3), result(4), result(5)};
  jaka_move_interface->linearMoveBasedTcp(pose_tcp_target, 10, 10, TRUE);
 
  return 0;
}