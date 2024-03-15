// -------------------------------
// NCNN SDK C 接口的公用头文件定义
//   2022 11 15
// -----------------------------
#ifndef _NCNN_C_SDK_COMMON_
#define _NCNN_C_SDK_COMMON_

#ifdef __cplusplus
	extern "C" {
#endif
// (1) 错误码
enum SDKErrorCode {
	OK = 0,               // Normal return
	PATH_ERROR = 1,       // You need to checkout filepath 
	PARAM_ERROR = 2,      // Unvalid function parameter exites
	OUT_OFME_MORY = 3,    // Out of memory 
	IMG_EMNPTY = 4,       // Image data is empty
	SDK_NOT_INITED = 5    // SDK is not intialed 
};

// (2) 图像编码格式
enum IMG_FORMAR{
	IMG_NV21 = 0,// YUV420  
	IMG_NV12 = 1,
	IMG_BGR = 2, // 
	IMG_RGB = 3,
	IMG_GRAY = 4
};

// (3) 检测结果
//#ifndef RectBox
typedef struct RectBox{
    float location[4];      //  位置 ，依次存储左上角左边x,y 和宽高 width,height 
    int label;              //  标签
    float score;            //  得分
}RectBox;
//#endif//RectBox

// (4) 分类结果
//#ifndef _ClassFicationResult
typedef struct _ClassFicationResult{
	int label;
	float prob;
}ClassFicationResult;
//#endif//ClassFicationResult

//（5）粗定位结果
//#ifndef rough_location
// typedef struct RoughLocation{
//     int classname;   // 目标类别
// 	float move;              // 终端沿滑轨滑动距离 单位mm
//     float distance;          // 滑台伸出距离 单位mm
// }RoughLocation;

#ifdef __cplusplus
}
#endif

#endif//_NCNN_C_SDK_COMMON_