/*********************************************
/   jiahou.wen@lotus.com.cn
/   latesed modified : 03/28/2023
*********************************************/

#ifndef _AHPC_DETECTOR_H_
#define _AHPC_DETECTOR_H_

#include<vector>
#include<string>
#include"SDK_common_define.h"
#include"ncnn_c_sdk_common.h"

class ahpc_detector
{
public:
	ahpc_detector(/* args */);
	~ahpc_detector();

	int init(const std::string& ini_file);
	int load_model();
	int detect(unsigned char* img_data, int img_format, unsigned int img_width, unsigned int img_height, RectBox** boxes, int* boxes_size);
	int release();
	 
private:

	long long pNcnnDetector_;//地址
    long long pRknnDetector_;
};

#endif//_AHPC_DETECTOR_H_