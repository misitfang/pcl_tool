#ifndef _SDK_COMMON_DEFINE_H_
#define _SDK_COMMON_DEFINE_H_

// (1) 检测结果
#ifndef RectBox
typedef struct RectBox {
	float location[4];      //  位置 ，依次存储左上角左边x,y 和宽高 width,height 
	int label;              //  标签
	float score;            // 
}RectBox;
#endif//RectBox

#endif
