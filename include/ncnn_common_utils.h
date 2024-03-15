#ifndef _NCNN_COMMON_UTILS_H_
#define _NCNN_COMMON_UTILS_H_
/*********************************************
/   jiahou.wen@lotus.com.cn
/   latesed modified : 07/26/2022
*********************************************/

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <vector>
#include<iostream>
#include "layer.h"
#include "net.h"
#include<algorithm>
#include<memory>

#include"SDK_common_define.h"

inline float intersection_area(const Object& a, const Object& b);
void qsort_descent_inplace(std::vector<Object>& faceobjects, int left, int right);
void qsort_descent_inplace(std::vector<Object>& faceobjects);
void nms_sorted_bboxes(const std::vector<Object>& faceobjects,
	std::vector<int>& picked, float nms_threshold);
inline float sigmoid(float x);
void generate_proposals(const ncnn::Mat& anchors, int stride, const ncnn::Mat& in_pad,
	const ncnn::Mat& feat_blob, float prob_threshold, std::vector<Object>& objects);
cv::Mat draw_objects(const cv::Mat& bgr, const std::vector<Object>& objects);

#endif//_NCNN_COMMON_UTILS_H_