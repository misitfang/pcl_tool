#include "so_test.h"

#include"yolov5s_ncnn_sdk.h"

#include <iostream>
#include<string>
#include<math.h>

#ifdef _WIN32
	#include <direct.h>
	#include <io.h>
#else
	#include <sys/time.h>
	#include <sys/io.h>
	#include <unistd.h>
	#include <sys/stat.h>
	#include <sys/types.h>
	#include <dirent.h>
#endif
// 检测框的标签
static const char* class_names[] = {
	"charger","circle1","circle2","circle3"
};

#define SHOW_IMGS
int run_images_test()
{
	std::string ini_path = "/home/ahpc/workspace/yolov5s_ncnn_0117/yolov5s_ncnn/config/charger.ini";
    std::string img_path = "/home/ahpc/workspace/yolov5s_ncnn_0117/yolov5s_ncnn/datas/charger0116/*";
	std::string out_path = "/home/ahpc/workspace/yolov5s_ncnn_0117/yolov5s_ncnn/datas/out_path/charger/";
	if (access(out_path.c_str(), 0) == -1) { //判断该文件夹是否存在
#ifdef _WIN32
    	int flag = mkdir(out_path.c_str());  //Windows创建文件夹
#else
    	int flag = mkdir(out_path.c_str(), S_IRWXU);  //Linux创建文件夹
#endif
		if (flag == 0) {  //创建成功
			std::cout << "Create directory successfully." << std::endl;
		} else { //创建失败
			std::cout << "Fail to create directory." << std::endl;
			throw std::exception();
		}
	}
    // 必须cv的String
    std::vector<std::string> fn;
    cv::glob(img_path, fn, false);
    size_t count = fn.size();
    std::cout << count << std::endl;
	double total_det_time = 0;
	double total_process_time = 0;
	int total_det_time_count = 0;

	long long pDetHandle = 0;// 检测器句柄
	int ret = creat_yolov5s_detector(&pDetHandle);
	if (ret != SDKErrorCode::OK)
	{
		std::cout << "creat_yolov5s_detector eror ! ret = " << ret << std::endl;
		return ret;//
	}
	std::cout << "creat_yolov5s_detector ok !\n";
	ret = init_detector(&pDetHandle, (char*)ini_path.c_str());
	std::cout << "init_detector ok !\n";
    
	if (ret != SDKErrorCode::OK)
	{
		std::cout << "init_detector erorr ! ret = " << ret << std::endl;;
		return ret;
	}

	RectBox* pMyBoxes = nullptr; //目标检测框数组
	int box_size = 0;
	int img_format = IMG_FORMAR::IMG_BGR;//

	std::cout << " my_files.size() = " << fn.size() << std::endl;
	struct timeval t1,t2,t3;
	double timeuse;
	double timeuse_ms;
	int fontface = cv::FONT_HERSHEY_SIMPLEX;
	for(int tt = 0;tt<fn.size();tt++)
	{
		std::cout << "total_det_time_count = " << total_det_time_count << std::endl;
		cv::Mat m =  cv::imread(fn[tt]);
		if (m.empty()){
			std::cout << " video is empty " << std::endl;
			return -1;
		}
		gettimeofday(&t1,NULL);
		int ret_det = detect(&pDetHandle, m.data, img_format, m.cols, m.rows, &pMyBoxes,&box_size); 
		// for(int i=0;i<box_size;i++){

		// std::cout<<"pMyBoxes["<<i<<"]:"<<pMyBoxes[i].location[0]<<" "<<pMyBoxes[i].location[1]<<" "<<
		// pMyBoxes[i].location[2]<<" "<<pMyBoxes[i].location[3]<<" "<<std::endl;
		// }

		gettimeofday(&t2,NULL);
    	timeuse = t2.tv_sec - t1.tv_sec + (t2.tv_usec - t1.tv_usec)/1000000.0;//
    	timeuse_ms=t2.tv_sec*1000.0 - t1.tv_sec*1000.0 + (t2.tv_usec - t1.tv_usec)/1000.0;
		auto t_used = timeuse_ms;//
		total_det_time += t_used;
		std::cout << " detect time used is :" << t_used << " ms " << std::endl;

#ifdef SHOW_IMGS
		cv::Mat image = m.clone();
		for (int i = 0; i < box_size; i++)
		{

			std::string label = class_names[pMyBoxes[i].label];

			std::cout<<"pMyBoxes["<<i<<"]:"<<pMyBoxes[i].location[0]<<" "<<pMyBoxes[i].location[1]<<" "<<
			pMyBoxes[i].location[2]<<" "<<pMyBoxes[i].location[3]<<"label: "<<label<<std::endl;

			std::string text = label + ":" + std::to_string(pMyBoxes[i].score);
			cv::Point l_t(int(pMyBoxes[i].location[0]), int(pMyBoxes[i].location[1]));
			cv::Point r_d(int(pMyBoxes[i].location[0] + pMyBoxes[i].location[2]), int(pMyBoxes[i].location[1]+pMyBoxes[i].location[3]));
			cv::Point l_d(int(pMyBoxes[i].location[0]), int(pMyBoxes[i].location[1]+pMyBoxes[i].location[3]));
			cv::rectangle(image, l_t, r_d, cv::Scalar(255, 5, 5));
			cv::putText(image,text,l_d,fontface,1,cv::Scalar(0, 0, 255),1,8);
		}
		gettimeofday(&t3,NULL);
    	timeuse = t3.tv_sec - t2.tv_sec + (t3.tv_usec - t2.tv_usec)/1000000.0;//
    	timeuse_ms=t3.tv_sec*1000.0 - t2.tv_sec*1000.0 + (t3.tv_usec - t2.tv_usec)/1000.0;
		t_used = timeuse_ms;//
		total_process_time += t_used;
		std::cout << " process time used is :" << t_used << " ms " << std::endl;
		cv::imshow("demo_charger", image);
		std::string out_name = out_path + "result_"+std::to_string(total_det_time_count)+".jpg";
		cv::imwrite(out_name,image);
		cv::waitKey(0);
#endif//#ifdef SHOW_IMGS
		total_det_time_count++;
	}
	std::cout << "average det time : " << (total_det_time+total_process_time) / total_det_time_count << std::endl;
	// cv::destroyAllWindows();
	release_yolov5s_detector(&pDetHandle);// 资源释放  
	return 0;
}

int main()
{
    print_test();
	run_images_test();
	std::cout << "Test ok !\n";
}
// 枝叶分割算法====================================================================================================
#include <iostream>
#include <pcl/io/pcd_io.h>  
#include <pcl/point_types.h>  
#include <pcl/common/pca.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/extract_clusters.h> // 欧式聚类
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>


using namespace std;
// 枝叶分割算法
std::vector<int> SegmentBanchAndLeaves(pcl::PointCloud<pcl::PointXYZ>::ConstPtr input_cloud, 
	float searchR = 0.1, float threshould = 0.8, float minDis = 0.06, int minPtNum = 10, int nt = 200)
{
	// ------------------------------------建立kd-tree索引---------------------------------
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(input_cloud);
	int K = 10;
	pcl::Indices r_indices;
	std::vector<float> neighbor_square_distance;
	
	pcl::Indices bole; // 存储树干点索引的容器
	for (size_t i = 0; i < input_cloud->points.size(); ++i)
	{
		// PCA的计算至少需要三个点
		if (kdtree.radiusSearch(input_cloud->points[i], 0.1, r_indices, neighbor_square_distance, K) > 3)
		{
			// --------------------------PCA计算每个点的特征值-----------------------------
			pcl::PCA<pcl::PointXYZ> pca;
			pca.setInputCloud(input_cloud);
			pca.setIndices(std::make_shared<const pcl::Indices>(r_indices));
			pca.getEigenValues(); // 获取特征值，特征值是按从大到小排列的

			float l1 = pca.getEigenValues()[0];
			float l2 = pca.getEigenValues()[1];
			float l3 = pca.getEigenValues()[2];
			// -------------------------计算每个点的散乱度指标-----------------------------
			float scatter = (l1 - l3) / l1;// 散乱度
			// ---------------------------设置阈值分离枝叶---------------------------------
			if (scatter > threshould)
			{
				bole.push_back(i);
			}
		}
		else
		{
			// 如果邻域范围内没有足够多的点，则跳过该点。
			continue;
		}

	}
	if (bole.empty())
	{
		cout << "失败！！！！" << endl;
		exit(0);
	}

	// -------------------------------------------欧式聚类--------------------------------------------
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(input_cloud);
	vector<pcl::PointIndices> cluster_indices; // 聚类索引
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;// 欧式聚类对象
	ec.setInputCloud(input_cloud);             // 设置输入点云
	ec.setIndices(std::make_shared<const pcl::Indices>(bole));                       // 枝干点的索引
	ec.setSearchMethod(tree);                  // 设置点云的搜索机制
	ec.setClusterTolerance(minDis);            // 设置近邻搜索的搜索半径（也即两个不同聚类团点之间的最小欧氏距离）
	ec.setMinClusterSize(minPtNum);            // 设置一个聚类需要的最少的点数目为100
	ec.setMaxClusterSize(25000);               // 设置一个聚类需要的最大点数目为25000
	ec.extract(cluster_indices);               // 从点云中提取聚类，并将点云索引保存在cluster_indices中
	
	std::vector<int>mBranch;
	for (auto it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
	{
		if (it->indices.size() > nt)
		{
			for (auto pit = it->indices.begin(); pit != it->indices.end(); pit++)
			{
				mBranch.push_back(*pit);
			}
		}
	}

	return mBranch;
}


int main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile<pcl::PointXYZ>("E://data//one1_tree.pcd", *cloud) == -1)
	{
		PCL_ERROR("Cloudn't read file!");
		return -1;
	}
	float searchRadius = 0.1; // 半径R
	float threshould = 0.8;   // 散乱度阈值
	float minDis = 0.06;      // 欧式聚类尺寸阈值
	int minPtNum = 10;        // 欧式聚类最小点数
	int nt = 200;             // 欧式聚类点数阈值
	//-----------------------------枝叶分离---------------------------------
	auto mBranchIdx = SegmentBanchAndLeaves(cloud, searchRadius, threshould, minDis, minPtNum, nt);
	
	pcl::ExtractIndices<pcl::PointXYZ> extr;
	extr.setInputCloud(cloud);//设置输入点云
	extr.setIndices(std::make_shared<const std::vector<int>>(mBranchIdx));//设置索引
	pcl::PointCloud<pcl::PointXYZ>::Ptr branchCloud(new pcl::PointCloud<pcl::PointXYZ>);
	extr.filter(*branchCloud);     //提取对应索引的点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr leafCloud(new pcl::PointCloud<pcl::PointXYZ>);
	extr.setNegative(true);
	extr.filter(*leafCloud);
	//-----------------------------保存树枝----------------------------------
	pcl::io::savePCDFileASCII("branchCloud.pcd", *branchCloud);
	//-----------------------------保存树叶----------------------------------
	pcl::io::savePCDFileASCII("leafCloud.pcd", *leafCloud);
	//---------------------------可视化结果----------------------------------
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("显示点云"));
	viewer->setWindowName("枝叶分离");
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addPointCloud<pcl::PointXYZ>(branchCloud, "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "sample cloud");
	viewer->addPointCloud<pcl::PointXYZ>(leafCloud, "cloud_filtered");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "cloud_filtered");
	//viewer->addCoordinateSystem(1.0);
	//viewer->initCameraParameters();
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
	return 0;
}

