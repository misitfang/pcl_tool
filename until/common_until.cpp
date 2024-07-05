
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Core>

/**
 * @brief 
 * input normal_vector      平面法向量
 * output rotation_vector   旋转向量

 */
void GetRotationVector(Eigen::Vector3d &normal_vector, Eigen::AngleAxisd &rotation_vector)
{
    LOG(INFO) << "start calculate rotation_vector-------------" << std::endl;

    //与那个轴直接的旋转。z轴
    Eigen::Vector3d z_axis(0, 0, 1);

    // 法向量和z轴叉乘得到旋转向量
    Eigen::Vector3d rotation_axis = z_axis.cross(normal_vector).normalized();

    double theta = acos(z_axis.dot(normal_vector) / (z_axis.norm() * normal_vector.norm()));

    // 点乘（0，0，1）（x,y,z)
    /* double denominator = sqrt(normal_vector[0] * normal_vector[0] + normal_vector[1] * normal_vector[1] + normal_vector[2] * normal_vector[2]);

    Eigen::Vector3d unit_normal_vector(-normal_vector[0] / denominator, -normal_vector[1] / denominator, normal_vector[1] / denominator);

    unit_normal_vector = unit_normal_vector.normalized();
    double theta = z_axis.dot(unit_normal_vector) / (z_axis.norm() * unit_normal_vector.norm()); */

    // LOG(INFO) << "z_axis.norm(): " << z_axis.norm() << std::endl;
    // LOG(INFO) << "unit_normal_vector.norm(): " << unit_normal_vector.norm() << std::endl;

    LOG(INFO) << "theta from point_ratationis : " << ((theta * 180) / 3.14) << std::endl;
    Eigen::AngleAxisd rotation_vector_tmp(theta, rotation_axis);
    rotation_vector = rotation_vector_tmp;
    LOG(INFO) << "finish calculate rotation_vector-------------" << std::endl;
}




// 获取当前时间并返回格式化后的字符串
std::string GetCurTime()
{
	time_t a = time(NULL);
	tm currtime = *localtime(&a);
	std::ostringstream Tm;
	Tm.fill('0');
	Tm << "Test_"
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
std::string tm_str;
tm_str = CURRENT_TIME();

//glog 快速初始化’
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

    log_info = log_info_path + "ICP_";
    FLAGS_alsologtostderr = true;
    FLAGS_colorlogtostderr = true;
    google::SetLogDestination(google::GLOG_INFO, log_info.c_str());
    google::SetLogFilenameExtension(".log");

    LOG(INFO) << "google::IsGoogleLoggingInitialized :" << google::IsGoogleLoggingInitialized() << std::endl;
    google::InitGoogleLogging("init logs------");
}
std::string log_info_path = "../data/logs/ICP/";
GolgInit(log_info_path);


//R t --->T
void GetTransformationMatrix(Eigen::Matrix3d &rotation_matrix, Eigen::Vector3d &translation_matrix,
                             Eigen::Matrix4d &transformation_atrix)
{

    Eigen::Isometry3d T_M3 = Eigen::Isometry3d::Identity();
    T_M3.rotate(rotation_matrix);
    T_M3.pretranslate(translation_matrix);
    transformation_atrix = T_M3.matrix();
}



/// get final result  
//R t --->T
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


    //三维点投影到2d图像
    Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(precise_charge_pose->yaw,Eigen::Vector3d::UnitX()));
    Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(precise_charge_pose->pitch,Eigen::Vector3d::UnitY()));
    Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(precise_charge_pose->roll+0.053156,Eigen::Vector3d::UnitZ()));
 
    Eigen::Matrix3d rotation_matrix;
    Eigen::Vector3d translation_matrix(precise_charge_pose->x,precise_charge_pose->y,precise_charge_pose->z);
    rotation_matrix=yawAngle*pitchAngle*rollAngle;
    Eigen::Isometry3d T_M3 = Eigen::Isometry3d::Identity();
    T_M3.rotate(rotation_matrix);
    T_M3.pretranslate(translation_matrix);
    transformation_atrix = T_M3.matrix();
    T=T_to_tcp.inverse()*transformation_atrix;
    pcl::PointCloud<pcl::PointXYZ>::Ptr center_trans_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    Eigen::Vector2d center_trans_uv;
    for(size_t i=0;i<kps3d_circles_center.size();i++)
    {
        pcl::PointXYZ point;
        point.x=kps3d_circles_center[i][0];
        point.y=kps3d_circles_center[i][1];
        point.z=kps3d_circles_center[i][2];
        center_trans_cloud->push_back(point);
    }
  	pcl::transformPointCloud(*center_trans_cloud, *center_trans_cloud, T.cast<float>());

    for(size_t i=0 ;i<center_trans_cloud->size();i++)
    {
        center_trans_uv[0]=1719.57*center_trans_cloud->points[i].x/center_trans_cloud->points[i].z +953.387;
        center_trans_uv[1]=1720.51*center_trans_cloud->points[i].y/center_trans_cloud->points[i].z +546;
        //保存至原图中
        cv::circle(img_mat, cv::Point2d(center_trans_uv[0],center_trans_uv[1]), 1, cv::Scalar(0, 0, 255), -1);
    }
	cv::imwrite("./test.jpg", img_mat);