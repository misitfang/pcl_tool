#ifndef _SDK_COMMON_DEFINE_H_
#define _SDK_COMMON_DEFINE_H_

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include<iostream>
#include<algorithm>
#include<memory>
#include"types.h"

#include"ncnn_c_sdk_common.h"

//enum SDKErrorCode {
//	OK = 0,               // Normal return
//	PATH_ERROR = 1,       // You need to checkout filepath 
//	PARAM_ERROR = 2,      // Unvalid function parameter exites
//	OUT_OFME_MORY = 3,    // Out of memory 
//	IMG_EMNPTY = 4,       // Image data is empty
//	SDK_NOT_INITED = 5    // SDK is not intialed 
//};

enum CarObjectType {
	Car = 0,
	CarTail = 1,
	EleTail = 2,
	CarFace = 3,
	LicensePlate = 4
};

enum CarTailType {
	OthersCarTail = 0,
	EletreTail = 1,
	TaycanTail = 2,
	BenzEqsFace = 3
};

#ifndef Object
typedef struct Object
{
	cv::Rect rect;
	int label;
	float prob;
	bool valid;

	Object(){
		rect = cv::Rect(0,0,0,0);
		label = -1;
		prob = 0.;
		valid = true;
	}
}Object;
#endif

#ifndef ObjectWithLandmark
typedef struct _ObjectWithLandmark{
	Object lpObj; // 目标框
	VIPLPoint landmarks[4];// 关键点
}ObjectWithLandmark;
#endif//_ObjectWithLandmark

//#ifndef ClassFicationResult
//typedef struct _ClassFicationResult
//{
//	int label;
//	float prob;
//}ClassFicationResult;
//#endif

#endif
