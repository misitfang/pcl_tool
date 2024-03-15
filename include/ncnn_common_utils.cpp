#include"ncnn_common_utils.h"

//class YoloV5Focus : public ncnn::Layer
//{
//public:
//	YoloV5Focus()
//	{
//		one_blob_only = true;
//	}
//	virtual int forward(const ncnn::Mat& bottom_blob, ncnn::Mat& top_blob, const ncnn::Option& opt) const
//	{
//		int w = bottom_blob.w;
//		int h = bottom_blob.h;
//		int channels = bottom_blob.c;
//		int outw = w / 2;
//		int outh = h / 2;
//		int outc = channels * 4;
//
//		top_blob.create(outw, outh, outc, 4u, 1, opt.blob_allocator);
//		if (top_blob.empty())
//			return -100;
//
//#pragma omp parallel for num_threads(opt.num_threads)
//		for (int p = 0; p < outc; p++)
//		{
//			const float* ptr = bottom_blob.channel(p % channels).row((p / channels) % 2) + ((p / channels) / 2);
//			float* outptr = top_blob.channel(p);
//
//			for (int i = 0; i < outh; i++)
//			{
//				for (int j = 0; j < outw; j++)
//				{
//					*outptr = *ptr;
//
//					outptr += 1;
//					ptr += 2;
//				}
//
//				ptr += w;
//			}
//		}
//
//		return 0;
//	}
//};
//DEFINE_LAYER_CREATOR(YoloV5Focus)

inline float intersection_area(const Object& a, const Object& b)
{
	cv::Rect_<float> inter = a.rect & b.rect;
	return inter.area();
}

void qsort_descent_inplace(std::vector<Object>& faceobjects, int left, int right)
{
	int i = left;
	int j = right;
	float p = faceobjects[(left + right) / 2].prob;
	while (i <= j)
	{
		while (faceobjects[i].prob > p)
			i++;

		while (faceobjects[j].prob < p)
			j--;

		if (i <= j)
		{
			// swap
			std::swap(faceobjects[i], faceobjects[j]);
			i++;
			j--;
		}
	}

#pragma omp parallel sections
	{
#pragma omp section
		{
			if (left < j) qsort_descent_inplace(faceobjects, left, j);
		}
#pragma omp section
		{
			if (i < right) qsort_descent_inplace(faceobjects, i, right);
		}
	}
}

void qsort_descent_inplace(std::vector<Object>& faceobjects)
{
	if (faceobjects.empty())
		return;

	qsort_descent_inplace(faceobjects, 0, faceobjects.size() - 1);
}

void nms_sorted_bboxes(const std::vector<Object>& faceobjects, std::vector<int>& picked, float nms_threshold)
{
	picked.clear();

	const int n = faceobjects.size();

	std::vector<float> areas(n);
	for (int i = 0; i < n; i++)
	{
		areas[i] = faceobjects[i].rect.area();
	}

	for (int i = 0; i < n; i++)
	{
		const Object& a = faceobjects[i];

		int keep = 1;
		for (int j = 0; j < (int)picked.size(); j++)
		{
			const Object& b = faceobjects[picked[j]];

			// intersection over union
			float inter_area = intersection_area(a, b);
			float union_area = areas[i] + areas[picked[j]] - inter_area;
			// float IoU = inter_area / union_area
			if (inter_area / union_area > nms_threshold)
				keep = 0;
		}

		if (keep)
			picked.push_back(i);
	}
}
 inline float sigmoid(float x)
{
	return static_cast<float>(1.f / (1.f + exp(-x)));
}

void generate_proposals(const ncnn::Mat& anchors, int stride, const ncnn::Mat& in_pad, const ncnn::Mat& feat_blob, float prob_threshold, std::vector<Object>& objects)
{
	const int num_grid = feat_blob.h;
	int num_grid_x;
	int num_grid_y;
	if (in_pad.w > in_pad.h)
	{
		num_grid_x = in_pad.w / stride;
		num_grid_y = num_grid / num_grid_x;
	}
	else
	{
		num_grid_y = in_pad.h / stride;
		num_grid_x = num_grid / num_grid_y;
	}

	const int num_class = feat_blob.w - 5;

	const int num_anchors = anchors.w / 2;

	for (int q = 0; q < num_anchors; q++)
	{
		const float anchor_w = anchors[q * 2];
		const float anchor_h = anchors[q * 2 + 1];

		const ncnn::Mat feat = feat_blob.channel(q);

		for (int i = 0; i < num_grid_y; i++)
		{
			for (int j = 0; j < num_grid_x; j++)
			{
				const float* featptr = feat.row(i * num_grid_x + j);

				// find class index with max class score
				int class_index = 0;
				float class_score = -FLT_MAX;
				for (int k = 0; k < num_class; k++)
				{
					float score = featptr[5 + k];
					if (score > class_score)
					{
						class_index = k;
						class_score = score;
					}
				}

				float box_score = featptr[4];

				float confidence = sigmoid(box_score) * sigmoid(class_score);

				if (confidence >= prob_threshold)
				{
					float dx = sigmoid(featptr[0]);
					float dy = sigmoid(featptr[1]);
					float dw = sigmoid(featptr[2]);
					float dh = sigmoid(featptr[3]);

					float pb_cx = (dx * 2.f - 0.5f + j) * stride;
					float pb_cy = (dy * 2.f - 0.5f + i) * stride;

					float pb_w = pow(dw * 2.f, 2) * anchor_w;
					float pb_h = pow(dh * 2.f, 2) * anchor_h;

					float x0 = pb_cx - pb_w * 0.5f;
					float y0 = pb_cy - pb_h * 0.5f;
					float x1 = pb_cx + pb_w * 0.5f;
					float y1 = pb_cy + pb_h * 0.5f;

					Object obj;
					obj.rect.x = x0;
					obj.rect.y = y0;
					obj.rect.width = x1 - x0;
					obj.rect.height = y1 - y0;
					obj.label = class_index;
					obj.prob = confidence;

					objects.push_back(obj);
				}
			}
		}
	}
}

cv::Mat draw_objects(const cv::Mat& bgr, const std::vector<Object>& objects)
{
	static const char* class_names[] = {
		"car", "car_tail", "eletre_tail", "taycan_tail"
	};
	cv::Mat image = bgr.clone();
	for (size_t i = 0; i < objects.size(); i++)
	{
		const Object& obj = objects[i];
		cv::rectangle(image, obj.rect, cv::Scalar(0, 0, 255));
		char text[256];
		sprintf(text, "%s %.1f%%", class_names[obj.label], obj.prob * 100);
		int baseLine = 0;
		cv::Size label_size = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);

		int x = obj.rect.x;
		int y = obj.rect.y - label_size.height - baseLine;
		if (y < 0)
			y = 0;
		if (x + label_size.width > image.cols)
			x = image.cols - label_size.width;

		cv::rectangle(image, cv::Rect(cv::Point(x, y), cv::Size(label_size.width, label_size.height + baseLine)),
			cv::Scalar(255, 0, 0), -1);

		cv::putText(image, text, cv::Point(x, y + label_size.height),
			cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0));
	}
	return image;
}