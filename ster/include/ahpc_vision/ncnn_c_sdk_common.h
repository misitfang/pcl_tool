#ifndef _NCNN_C_SDK_COMMON_
#define _NCNN_C_SDK_COMMON_

#ifdef __cplusplus
	extern "C" {
#endif

// (1) 图像编码格式
typedef enum {
	IMG_NV21 = 0,// YUV420  
	IMG_NV12 = 1,
	IMG_BGR = 2,
	IMG_RGB = 3,
	IMG_GRAY = 4,
}frame_format_t;

//（2）图像帧信息
typedef struct{
	frame_format_t img_format; // 图像格式
	unsigned char* img_data;   // 图像数据指针
	unsigned int img_width;    // 图像宽度
	unsigned int img_height;   // 图像高度
}frame_info_t;


// (3) 通用检测器错误码
enum SDKErrorCode {
	OK = 0,               // Normal return
	PATH_ERROR = 1,       // You need to checkout filepath 
	PARAM_ERROR = 2,      // Unvalid function parameter exites
	OUT_OF_MEMORY = 3,    // Out of memory 
	IMG_EMNPTY = 4,       // Image data is empty
	SDK_NOT_INITED = 5    // SDK is not intialed 
};

#ifdef __cplusplus
}

#endif

#endif//_NCNN_C_SDK_COMMON_