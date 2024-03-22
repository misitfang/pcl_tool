#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/search/kdtree.h>
#include <pcl/common/transforms.h>
#include <glog/logging.h>
#include <Eigen/Core>
#include <pcl/visualization/pcl_visualizer.h>
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> Vector3fVector;

void ShowIcpViewer(PointCloudT::Ptr &cloud_soure_raw, PointCloudT::Ptr &cloud_target, PointCloudT::Ptr &cloud_source_trans)
{

	boost::shared_ptr<pcl::visualization::PCLVisualizer>
		viewer1(new pcl::visualization::PCLVisualizer("icp Viewer,red cloud_soure_raw,green cloud_target,blue cloud_source_trans "));
	viewer1->setBackgroundColor(0.5, 0.5, 0.5);

	pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_tr_color_h(cloud_soure_raw, 255, 0, 0);
	viewer1->addPointCloud(cloud_soure_raw, cloud_tr_color_h, "cloud_soure_raw"); //  设置charger_port_cloud点云为红色

	pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_tr_source_h(cloud_target, 0, 255, 0);
	viewer1->addPointCloud(cloud_target, cloud_tr_source_h, "cloud_target"); //  设置template_cloud点云为绿色

	pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_icp_color_h(cloud_source_trans, 0, 0, 255);
	viewer1->addPointCloud(cloud_source_trans, cloud_icp_color_h, "cloud_source_trans"); // icp_cloud结果显示蓝色

	viewer1->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud_soure_raw");
	viewer1->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud_target");
	viewer1->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud_source_trans");
	viewer1->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 2, "cloud_source_trans");

	viewer1->addCoordinateSystem(0.1);

	while (!viewer1->wasStopped())
	{
		viewer1->spinOnce(100);
		// icp.align(*template_cloud);
		// boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
}

void GolgInit(std::string &log_info_path)
{
	std::string log_info;
	DIR *dir;
	std::string str_command;
	if ((dir = opendir(log_info_path.c_str())) == NULL)
	{
		LOG(INFO) << "日志存放目录不存在，创建日志存放目录文件" << std::endl;
		str_command = "mkdir -p " + log_info_path;
		system(str_command.c_str()); // 存在就删除目录
	}

	log_info = log_info_path + "ICP_";
	FLAGS_alsologtostderr = true;
	FLAGS_colorlogtostderr = true;
	google::SetLogDestination(google::GLOG_INFO, log_info.c_str());
	google::SetLogFilenameExtension(".log");

	LOG(INFO) << "google::IsGoogleLoggingInitialized :" << google::IsGoogleLoggingInitialized() << std::endl;
	google::InitGoogleLogging("init logs------");
}
void GetTransformationMatrix(Eigen::Matrix3f &rotation_matrix, Eigen::Vector3f &translation_matrix,
							 Eigen::Matrix4f &transformation_matrix)
{

	Eigen::Isometry3f T_M3 = Eigen::Isometry3f::Identity();
	T_M3.rotate(rotation_matrix);
	T_M3.pretranslate(translation_matrix);
	transformation_matrix = T_M3.matrix();
}
void GetCentroid(Vector3fVector &v_source, Vector3fVector &v_target,
				 Eigen::Vector3f &centroid_source, Eigen::Vector3f &centroid_target,
				 Eigen::Matrix3f &W)
{
	for (size_t i = 0; i < v_source.size(); i++)
	{
		centroid_source[0] += v_source[i][0];
		centroid_source[1] += v_source[i][1];
		centroid_source[2] += v_source[i][2];
		centroid_target[0] += v_target[i][0];
		centroid_target[1] += v_target[i][1];
		centroid_target[2] += v_target[i][2];
	}
	centroid_source /= v_source.size();
	centroid_target /= v_target.size();
	LOG(INFO) << "centroid_source:" << centroid_source.transpose() << std::endl;
	LOG(INFO) << "centroid_target:" << centroid_target.transpose() << std::endl;

	Eigen::Vector3f de_centroid_source;
	Eigen::Vector3f de_centroid_target;
	de_centroid_source.setZero();
	de_centroid_target.setZero();
	// 去中心化点，p'=pi-p^ q'=qi-q^
	for (size_t i = 0; i < v_source.size(); i++)
	{
		de_centroid_source[0] = (v_source[i][0] - centroid_source[0]);
		de_centroid_source[1] = (v_source[i][1] - centroid_source[1]);
		de_centroid_source[2] = (v_source[i][2] - centroid_source[2]);
		de_centroid_target[0] = (v_target[i][0] - centroid_target[0]);
		de_centroid_target[1] = (v_target[i][1] - centroid_target[1]);
		de_centroid_target[2] = (v_target[i][2] - centroid_target[2]);
		W += de_centroid_source * de_centroid_target.transpose();
	}
	LOG(INFO) << "W:\n"
			  << W << std::endl;
}

void GetCentroid(Vector3fVector &vect_in,
				 Eigen::Matrix<float, 4, 1> &centroid,
				 Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> &deam_mattix_out)
{
	Eigen::Matrix<float, 4, 1> accumulator{0, 0, 0, 0};

	unsigned int cp = 0;

	// For each point in the cloud
	// If the data is dense, we don't need to check for NaN
	for (size_t i = 0; i < vect_in.size(); i++)
	{
		// Check if the point is invalid
		if (pcl::isFinite(*cloud_iterator))
		{
			accumulator[0] += vect_in[i][0];
			accumulator[1] += vect_in[i][1];
			accumulator[2] += vect_in[i][2];
			++cp;
		}
	}

	if (cp > 0)
	{
		centroid = accumulator;
		centroid /= static_cast<float>(cp);
		centroid[3] = 1;
	}

	// 计算与质心差值对均值
	std::size_t npts = vect_in.size();

	deam_mattix_out = Eigen::Matrix<Scalar, 4, Eigen::Dynamic>::Zero(4, npts); // keep the data aligned

	for (std::size_t i = 0; i < npts; ++i)
	{
		deam_mattix_out(0, i) = vect_in[i][0] - centroid[0];
		deam_mattix_out(1, i) = vect_in[i][1] - centroid[1];
		deam_mattix_out(2, i) = vect_in[i][2] - centroid[2];
		// One column at a time
		// cloud_out.block<4, 1> (0, i) = cloud_in[i].getVector4fMap () - centroid;
	}
}
int main()
{
	std::string log_info_path = "../data/logs/ICP/";
	GolgInit(log_info_path);
	// 定义点云数据
	PointCloudT::Ptr cloud_source(new PointCloudT());
	PointCloudT::Ptr cloud_source_in(new PointCloudT());
	PointCloudT::Ptr cloud_target(new PointCloudT());
	PointCloudT::Ptr cloud_source_new(new PointCloudT());
	PointCloudT::Ptr cloud_target_new(new PointCloudT());
	PointCloudT::Ptr cloud_out(new PointCloudT());

	pcl::Indices k_indices(1);
	std::vector<float> k_sqr_distances;
	pcl::Indices k_indices_reverse(1);
	std::vector<float> k_sqr_distances_reverse;
	float sqr_distances_th = std::numeric_limits<float>::max();
	sqr_distances_th = 1;
	float error_distance = std::numeric_limits<float>::infinity();
	LOG(INFO) << "sqr_distances_th :" << sqr_distances_th;
	LOG(INFO) << "error_distance :" << error_distance;
	Eigen::Matrix4f transformation_matrix = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f cur_transformation_matrix = Eigen::Matrix4f::Identity();
	// load point cloud data
	pcl::io::loadPCDFile("../data/pcd/icp_process/162755.pcd", *cloud_source_in);
	pcl::io::loadPCDFile("../data/pcd/icp_process/162755_trans.pcd", *cloud_target);
	LOG(INFO) << "cloud_source_in size :" << cloud_source_in->size() << std::endl;
	LOG(INFO) << "cloud_target size :" << cloud_target->size() << std::endl;
	Vector3fVector v_source, v_target;
	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
	pcl::search::KdTree<PointT>::Ptr tree_reverse(new pcl::search::KdTree<PointT>());
	transformation_matrix << 0.982161, 0.174716, -0.0695687, 0.0494402,
		-0.180878, 0.978886, -0.0952259, -0.00924198,
		0.0514619, 0.10611, 0.993024, -0.1433,
		0, 0, 0, 1;

	// transformation_matrix << 0, 1, 0, 0,
	// 	-1, 0, 0, 0,
	// 	0, 0, 1, 0,
	// 	0, 0, 0, 1;

	tree->setInputCloud(cloud_target);
	for (int iter = 0; iter < 50; iter++)
	{

		// Rp+t  转换source点云
		pcl::transformPointCloud(*cloud_source_in, *cloud_source, transformation_matrix);
		//
		tree_reverse->setInputCloud(cloud_source);
		v_source.clear();
		v_target.clear();
		cloud_source_new->clear();
		cloud_target_new->clear();
		// 遍历source点云
		float sum_sqr_dis = 0.0;
		int count = 0;
		for (int i = 0; i < cloud_source->size(); i++)
		{
			tree->nearestKSearch(cloud_source->points[i], 1, k_indices, k_sqr_distances);
			// 最近点距离小于阈值
			if (k_sqr_distances[0] < sqr_distances_th)
			{
				// LOG(INFO) << "k_sqr_distances[0] :" << k_sqr_distances[0];
				// v_source.push_back(Eigen::Vector3f(cloud_source->points[i].x, cloud_source->points[i].y, cloud_source->points[i].z));
				// v_target.push_back(Eigen::Vector3f(cloud_target->points[k_indices[0]].x, cloud_target->points[0].y, cloud_target->points[0].z));

				// LOG(INFO) << "k_sqr_distances[0] :" << k_sqr_distances[0] << std::endl;
				// LOG(INFO) << "k_indices[0] :" << k_indices[0] << std::endl;
				tree_reverse->nearestKSearch(cloud_target->points[k_indices[0]], 1, k_indices_reverse, k_sqr_distances_reverse);
				if (k_indices_reverse[0] == i)
				{
					LOG(INFO) << "k_indices_reverse[0] :" << k_indices_reverse[0] << std::endl
							  << "i :" << i << "互为最近点";
					v_source.push_back(Eigen::Vector3f(cloud_source->points[i].x, cloud_source->points[i].y, cloud_source->points[i].z));
					v_target.push_back(Eigen::Vector3f(cloud_target->points[k_indices[0]].x, cloud_target->points[0].y, cloud_target->points[0].z));
					cloud_source_new->points.push_back(cloud_source->points[i]);
					cloud_target_new->points.push_back(cloud_target->points[k_indices[0]]);
					sum_sqr_dis += k_sqr_distances[0];
					count++;
				}
			}
		}

		// 保存提取的点对用作显示
		//  cloud_source_new->height = 1;
		//  cloud_source_new->width =cloud_source_new->points.size() ;
		//  pcl::io::savePCDFile("./cloud_source_new.pcd", *cloud_source_new);
		//  cloud_target_new->height = 1;
		//  cloud_target_new->width =cloud_target_new->points.size() ;
		//  pcl::io::savePCDFile("./cloud_source_new.pcd", *cloud_source_new);
		//  pcl::io::savePCDFile("./cloud_target_new.pcd", *cloud_target_new);
		LOG(INFO) << "count:" << count << std::endl;
		// 对应点之间平方和距离平均值
		error_distance = (sum_sqr_dis) / count;
		sqr_distances_th = error_distance * 2;
		LOG(INFO) << "error_distance:" << error_distance << std::endl;
		LOG(INFO) << "sqr_distances_th:" << sqr_distances_th << std::endl;
		LOG(INFO) << "v_source.size():" << v_source.size() << std::endl;
		LOG(INFO) << "v_target.size():" << v_target.size() << std::endl;
		LOG(INFO) << "cloud_source_new->points.size():" << cloud_source_new->points.size() << std::endl;
		LOG(INFO) << "cloud_target_new->points.size():" << cloud_target_new->points.size() << std::endl;

		// 计算点云点集质心 p^ q^
		Eigen::Vector3f centroid_source;
		Eigen::Vector3f centroid_target;
		centroid_source.setZero();
		centroid_target.setZero();
		Eigen::Matrix3f W;
		W.setZero();
		GetCentroid(v_source, v_target, centroid_source, centroid_target, W);

		Eigen::Matrix<float, 4, 1> centroid_source_mat;
		Eigen::Matrix<float, 4, 1> centroid_target_mat;
		Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> deam_matrix_out;
		Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> deam_matrix_out;
			GetCentroid(v_source, cloud_target_new, centroid_source, centroid_target, W);
		// w=P’q'^T
		Eigen::Matrix<Scalar, 3, 3> H = (cloud_src_demean * cloud_tgt_demean.transpose()).topLeftCorner(3, 3);
		Eigen::JacobiSVD<Eigen::Matrix3f> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
		// svd 求解  R=VU^T   t=q^-Rp^
		Eigen::Matrix3f V = svd.matrixV();
		Eigen::Matrix3f U = svd.matrixU();
		Eigen::Matrix3f R = V * U.transpose();
		LOG(INFO) << "R:\n"
				  << R << std::endl;
		Eigen::Vector3f t = centroid_target - R * centroid_source;
		LOG(INFO) << "t:" << t.transpose() << std::endl;
		GetTransformationMatrix(R, t, cur_transformation_matrix);
		LOG(INFO) << "cur transformation_matrix:\n"
				  << cur_transformation_matrix << std::endl;
		transformation_matrix = cur_transformation_matrix * transformation_matrix;
		LOG(INFO) << "final transformation_matrix:\n"
				  << transformation_matrix << std::endl;
		ShowIcpViewer(cloud_source_in, cloud_target, cloud_source);
	}
	pcl::transformPointCloud(*cloud_source_in, *cloud_source, transformation_matrix);
	// ShowIcpViewer(cloud_source_in, cloud_target, cloud_source);
	google::ShutdownGoogleLogging();
	return 0;
}