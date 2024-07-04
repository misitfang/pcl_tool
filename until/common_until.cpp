
#include <iostream>



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
