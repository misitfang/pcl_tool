#ifndef _YOLOV5S_NCNN_SDK_H_
#define _YOLOV5S_NCNN_SDK_H_

#ifdef __cplusplus
extern "C" {
#endif

#include"ncnn_c_sdk_common.h"
// 创建检测器实例
// - pHandle 检测器句柄
int creat_yolov5s_detector(long long* pHandle);

// 检测器初始化
// - pHandle 检测器句柄
// - ini_path 算法配置文件
int init_detector(long long * pHandle, char* ini_path);

// // 检测器初始化
// // - pHandle 检测器句柄
// // - image 预处理图像数据指针
// int pre_process(long long *pHandle, unsigned char* image);

// 执行目标检测任务
//@ - pHandle 检测器句柄
//@ - img_data 图像数据指针
//@ - img_format 图像格式
//@ - img_width 图像宽度
//@ - img_height 图像高度
//@ - boxes  检测结果指针
//@ - boxes_size  检测到的目标的个数
int detect(long long *pHandle,unsigned char* img_data, int img_format,unsigned int img_width,
            unsigned int img_height,RectBox** boxes,int* boxes_size);

//--- 执行后处理任务 ---
// - pHandle 检测器句柄
// - rough_location 粗定位结果
// - img_h 图像高度
// int post_process(long long *pHandle, RoughLocation** rough_location, unsigned int img_w, int* rough_size);

/// @brief  
/// @param camera_id   
/// @param image_width 
/// @param image_height 
/// @param image_fps 
/// @param data : out : BGR  image date 
/// @return 
int get_video_images(long long *pHandle,int camera_id,int image_width,int image_height,int image_fps,unsigned char* data);//

// 释放检测器资源
int release_yolov5s_detector(long long* pHandle);

#ifdef __cplusplus
}
#endif

#endif//_YOLOV5S_NCNN_SDK_H_