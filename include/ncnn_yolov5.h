/*********************************************
/   Kangli.hu1@lotus.com.cn
/   latesed modified : 01/16/2023
*********************************************/

#ifndef _NCNN_YOLOV5_H_
#define _NCNN_YOLOV5_H_

#include"ncnn_common_utils.h"
#include"INIReader.h"

#include"cpu.h"
#include"ncnn_c_sdk_common.h"

class Yolov5sDetector
{
public:
	Yolov5sDetector();
	~Yolov5sDetector();
	int init(const std::string& ini_file);
	int load_model(const std::string& param_path, const std::string& bin_path); //  从配置文件中获取绝对路径
	int load_model(const std::string& model_path);								//  从配置文件中获取件名
	int load_model();															//  从配置文件中获取路径和文件名
	std::vector<Object> detect(const cv::Mat& img_bgr, const float& prob_threshold);// 直接输入阈值
	std::vector<Object> detect(const cv::Mat& img_bgr);							// 采用配置文件的阈值
	
	// suport c interfaces
	int detect(const unsigned char* img_data,int img_format, unsigned int img_w, unsigned int img_h, RectBox** boxes, int* boxes_size);

	//粗定位后处理
	// int post_processing(RoughLocation** rough_location, unsigned int img_w, int* rough_size);

	int release();

	// 获取图像
	//int get_image_from_camera(int camera_id,int image_width,int image_height,int image_fps,unsigned char * bgr_date);

private:
	//目标检测参数
	int target_size_;
	float prob_threshold_;
	float nms_threshold_;
	int img_w_;
	int img_h_;

	//粗定位参数
	// float yaw_;  //摄像头水平偏转角
	// int32_t move_x_;  //摄像头到机械臂中心沿滑轨方向的水平距离
	// int32_t fx_;      //摄像头x轴方向焦距
	// int32_t fy_;      //摄像头y轴方向焦距

	ncnn::Net yolov5_;//
	ncnn::Mat in_;
	ncnn::Mat in_pad_;
	std::vector<ncnn::Mat> all_anchors_;//
	std::vector < std::string> outlayernames_;//

	// cv::Mat img_captured_;// 临时缓存
	// 多种图像格式支持

	float norm_vals_[3];
	int class_num_;
	int anchor_num_;
	std::vector<Object> proposals_;

	bool video_model_;// 1  : video  0 : imgs 
	bool is_first_frame_;//首帧图像
	unsigned char* data_bgr_;// BGR格式图像

	std::string ncnn_param_name_;
	std::string ncnn_bin_name_;
	std::string model_root_name_;// 模型文件存放主路径

	// 模型推理引擎设置
	int inference_engine_;     // 推理引擎  0: cpu  1: vulkan  2 : GPU  default : cpu
	int num_of_thread_;		   // 线程数  

	// 缓存当前检测结果
	std::vector<RectBox> current_results_;
	void config_initialise(const std::string& ini_file);

	// 缓存粗定位结果
	// std::vector<RoughLocation> rough_location_results_;

	////临时图像抓取类
	//bool use_capture_;// 释放采用内置图像流程
	//cv::VideoCapture  capture_;
	//bool capture_start_;
};

#endif//_NCNN_YOLOV5_H_