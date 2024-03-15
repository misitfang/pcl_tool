#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/registration/icp.h> // icp算法
#include <pcl/visualization/pcl_visualizer.h>
// #include <boost/thread/thread.hpp>

using namespace std;

void cloud_with_normal(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointCloud<pcl::PointNormal>::Ptr& cloud_normals)
{
    //-----------------拼接点云数据与法线信息---------------------
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> n;//OMP加速
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    //建立kdtree来进行近邻点集搜索
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    n.setNumberOfThreads(10);//设置openMP的线程数
    //n.setViewPoint(0,0,0);//设置视点，默认为（0，0，0）
    n.setInputCloud(cloud);
    n.setSearchMethod(tree);
    n.setKSearch(10);//点云法向计算时，需要所搜的近邻点大小
    //n.setRadiusSearch(0.03);//半径搜素
    n.compute(*normals);//开始进行法向计
    //将点云数据与法向信息拼接
    pcl::concatenateFields(*cloud, *normals, *cloud_normals);
}
void  visualize_registration(pcl::PointCloud<pcl::PointXYZ>::Ptr& source,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& target, pcl::PointCloud<pcl::PointXYZ>::Ptr& icp)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("配准结果"));
    int v1 = 0;
    int v2 = 1;
    viewer->createViewPort(0, 0, 0.5, 1, v1);
    viewer->createViewPort(0.5, 0, 1, 1, v2);
    viewer->setBackgroundColor(0, 0, 0, v1);
    viewer->setBackgroundColor(0.05, 0, 0, v2);
    viewer->addText("Raw point clouds", 10, 10, "v1_text", v1);
    viewer->addText("Registed point clouds", 10, 10, "v2_text", v2);
    //原始点云绿色
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> src_h(source, 255, 0, 0);
    //目标点云蓝色
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> tgt_h(target, 0, 0, 255);
    //转换后的源点云红色
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transe(icp, 255, 0, 0);
    //viewer->setBackgroundColor(255, 255, 255);
    viewer->addPointCloud(source, src_h, "source cloud", v1);
    viewer->addPointCloud(target, tgt_h, "target cloud", v1);
    viewer->addPointCloud(target, tgt_h, "target cloud1", v2);
    viewer->addPointCloud(icp, transe, "pcs cloud", v2);
    //添加坐标系
    //viewer->addCoordinateSystem(0.1);
    //viewer->initCameraParameters();
    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        // boost::this_thread::sleep(boost::posix_time::microseconds(10000));
    }
}
int main()
{
    // --------------------加载源点云-----------------------
    pcl::PointCloud<pcl::PointXYZ>::Ptr source(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile<pcl::PointXYZ>("../models/pcl_G9_model.pcd", *source);
    cout << "从源点云中读取 " << source->size() << " 个点" << endl;
    // -------------------加载目标点云----------------------
    pcl::PointCloud<pcl::PointXYZ>::Ptr target(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile<pcl::PointXYZ>("../models/total_points_process.pcd", *target);
    cout << "从目标点云中读取 " << target->size() << " 个点" << endl;
    //-----------------拼接点云与法线信息-------------------
    pcl::PointCloud<pcl::PointNormal>::Ptr source_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    cloud_with_normal(source, source_with_normals);
    pcl::PointCloud<pcl::PointNormal>::Ptr target_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    cloud_with_normal(target, target_with_normals);
    //----------------点到面的icp（经典版）-----------------
    pcl::IterativeClosestPointWithNormals<pcl::PointNormal, pcl::PointNormal>p_icp;
    
   
    p_icp.setInputSource(source_with_normals);
    p_icp.setInputTarget(target_with_normals);
    p_icp.setTransformationEpsilon(1e-10);    // 为终止条件设置最小转换差异
    p_icp.setMaxCorrespondenceDistance(1);   // 设置对应点对之间的最大距离（此值对配准结果影响较大）。
    p_icp.setEuclideanFitnessEpsilon(0.001);  // 设置收敛条件是均方误差和小于阈值， 停止迭代；
    //p_icp.setUseSymmetricObjective(true);   // 设置为true则变为另一个算法
    p_icp.setMaximumIterations(35);           // 最大迭代次数
    pcl::PointCloud<pcl::PointNormal>::Ptr p_icp_cloud(new pcl::PointCloud<pcl::PointNormal>);
    p_icp.align(*p_icp_cloud);
    cout << "\nICP has converged, score is " << p_icp.getFitnessScore() << endl;
    cout << "变换矩阵：\n" << p_icp.getFinalTransformation() << endl;
    // 使用创建的变换对为输入的源点云进行变换
    pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*source, *out_cloud, p_icp.getFinalTransformation());
    pcl::io::savePCDFileASCII("667.pcd", *out_cloud);

    visualize_registration(source, target, out_cloud);

    system("pause");
    return (0);
}

