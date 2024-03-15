#pragma once
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

class ICP_Process
{
public:
	void setInputSource(PointCloudT::Ptr& cloud);    // 输入源点云
	void setInputTarget(PointCloudT::Ptr& cloud);    // 输入目标点云
	void setMaximumIterations(int maxIters);                            // 最大迭代次数
	void setEuclideanFitnessEpsilon(float errorTh);                     // 误差阈值
	void align(Eigen::Matrix4f& RTMatrix);                              // ICP
	
	~ICP_Process() {}

private:
	int m_maxIters = 50;       // 最大迭代次数默认参数
	float m_errorTh = 0.0001;  // 误差阈值
	
	PointCloudT::Ptr m_source; // 源点云
	PointCloudT::Ptr m_target; // 目标点云
};

