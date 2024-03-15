#include "icp_process.h"
#include <pcl/io/pcd_io.h> 
#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <vector>
#include <iostream>
#include <pcl/search/kdtree.h> 


// 设置输入源点云
void ICP_Process::setInputSource(PointCloudT::Ptr& cloud)
{
	m_source = cloud;
};
// 设置输入目标点云
void ICP_Process::setInputTarget(PointCloudT::Ptr& cloud)
{
	m_target = cloud;
};
// 设置最大迭代次数
void ICP_Process::setMaximumIterations(int maxIters)
{
	m_maxIters = maxIters;
};
// 设置收敛误差
void ICP_Process::setEuclideanFitnessEpsilon(float errorTh)
{
	m_errorTh = errorTh;
};
// ICP计算过程
void ICP_Process::align(Eigen::Matrix4f& RTMatrix)
{
	// ----------------------------------建立kd树----------------------------------
	pcl::search::KdTree<PointT>::Ptr kdtree(new pcl::search::KdTree<PointT>);
	kdtree->setInputCloud(m_target);
	std::vector<int>pointIdxNKNSearch(1);
	std::vector<float>pointNKNSquaredDistance(1);
	PointT searchPoint = { 0.0,0.0,0.0 };  // 初始化查找点
   // 初始化变换矩阵、中间变换数据等参数
	int iters = 0;
	float error = std::numeric_limits<float>::infinity();
	Eigen::Matrix3f Rota = Eigen::Matrix3f::Identity();
	Eigen::Vector3f Trans = Eigen::Vector3f::Zero();
	Eigen::Matrix4f RT = Eigen::Matrix4f::Identity();
	RTMatrix = Eigen::Matrix4f::Identity();

	PointCloudT::Ptr source_mid(new PointCloudT);
	PointCloudT::Ptr target_mid(new PointCloudT);
	pcl::copyPointCloud(*m_source, *source_mid);
	// --------------------------------ICP迭代过程----------------------------------
	while (error > m_errorTh && iters < m_maxIters)
	{
		iters++;
		float lastError = error;
		float err = 0.0;
		pcl::transformPointCloud(*source_mid, *source_mid, RT);
		std::vector<int>indexs(source_mid->size());
		// 使用K近邻搜索获取最近邻点对
		for (int i = 0; i < source_mid->size(); ++i)
		{
			searchPoint = source_mid->points[i];
			kdtree->nearestKSearch(source_mid->points[i], 1, pointIdxNKNSearch, pointNKNSquaredDistance);
			err = err + sqrt(pointNKNSquaredDistance[0]);
			indexs[i] = pointIdxNKNSearch[0];
		}
		pcl::copyPointCloud(*m_target, indexs, *target_mid);
		error = err / m_source->size();
		// 前后两次迭代的误差小于1e-6，则停止迭代
		if (fabs(lastError - error) < 1e-6)
		{
			break;
		}
		// 1、计算点云中心坐标
		Eigen::Vector4f source_centroid, target_centroid_mid;
		pcl::compute3DCentroid(*source_mid, source_centroid);
		pcl::compute3DCentroid(*target_mid, target_centroid_mid);
		// 2、去中心化
		Eigen::MatrixXf souce_cloud_demean, target_cloud_demean;
		pcl::demeanPointCloud(*source_mid, source_centroid, souce_cloud_demean);
		pcl::demeanPointCloud(*target_mid, target_centroid_mid, target_cloud_demean);
		// 3、计算W=q1*q2^T
		Eigen::MatrixXf W = (souce_cloud_demean * target_cloud_demean.transpose()).topLeftCorner(3, 3);
		// 4、SVD分解得到新的旋转矩阵和平移矩阵
		Eigen::JacobiSVD<Eigen::MatrixXf> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
		Eigen::MatrixXf U = svd.matrixU();
		Eigen::MatrixXf V = svd.matrixV();

		if (U.determinant() * V.determinant() < 0)
		{
			for (int x = 0; x < 3; ++x)
				V(x, 2) *= -1;
		}

		Rota = V * U.transpose();
		Trans = target_centroid_mid.head(3) - Rota * source_centroid.head(3);
		RT << Rota, Trans, 0, 0, 0, 1;
		// 5、更新变换矩阵	
		RTMatrix = RT * RTMatrix;
	}
}



int main(int argc, char** argv)
{
	// --------------------------------读取源点云------------------------------------
	PointCloudT::Ptr source(new PointCloudT);
	if (pcl::io::loadPCDFile<PointT>("../data/icp_process/eletre.pcd", *source) == -1)
	{
		PCL_ERROR("couldn't read source file!");
		return -1;
	}
	// -------------------------------读取目标点云------------------------------------
	PointCloudT::Ptr target(new PointCloudT);
	if (pcl::io::loadPCDFile<PointT>("../data/icp_process/chargePortCLoud.pcd", *target) == -1)
	{
		PCL_ERROR("couldn't read target file!");
		return -1;
	}
	// ---------------------------------ICP配准---------------------------------------
	Eigen::Matrix4f transformation_matrix = Eigen::Matrix4f::Identity();
	ICP_Process icp;
	icp.setInputSource(source);
	icp.setInputTarget(target);
	icp.setMaximumIterations(35);
	icp.setEuclideanFitnessEpsilon(0.001);
	icp.align(transformation_matrix);
	std::cout << "变换矩阵为:\n" << transformation_matrix << std::endl;
	// ---------------------------------配准结果-------------------------------------
	PointCloudT::Ptr icp_cloud(new PointCloudT);
	pcl::transformPointCloud(*source, *icp_cloud, transformation_matrix);
	pcl::io::savePCDFileBinary("icp_cloud.pcd", *icp_cloud);
	bool show_icp=true;
	if(show_icp){

		// --------------------------------结果可视化-----------------------------------
		boost::shared_ptr<pcl::visualization::PCLVisualizer>viewer(new pcl::visualization::PCLVisualizer("ICP配准结果"));
		viewer->setBackgroundColor(0, 0, 0);  //设置背景颜色为黑色
		viewer->setWindowName("ICP配准结果");
		// 对目标点云着色可视化 (red).
		pcl::visualization::PointCloudColorHandlerCustom<PointT>target_color(target, 255, 0, 0);
		viewer->addPointCloud<PointT>(target, target_color, "target cloud");
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "target cloud");
		// 对源点云着色可视化 (blue).
		pcl::visualization::PointCloudColorHandlerCustom<PointT>source_color(source, 0, 0, 255);
		viewer->addPointCloud<PointT>(source, source_color, "source cloud");
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,1, "source cloud");
		// 对转换后的源点云着色 (green)可视化.
		pcl::visualization::PointCloudColorHandlerCustom<PointT>icp_color(icp_cloud, 0, 255, 0);
		viewer->addPointCloud<PointT>(icp_cloud, icp_color, "icp cloud");
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "icp cloud");
		// 启动可视化
		// viewer->addCoordinateSystem(0.1);  //显示XYZ指示轴
		// viewer->initCameraParameters();   //初始化摄像头参数
		while (!viewer->wasStopped())
		{
			viewer->spinOnce(100);
			// boost::this_thread::sleep(boost::posix_time::microseconds(100000));
		}
	}

	
	return 0;
}


