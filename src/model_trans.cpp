#include <iostream>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <Eigen/Core>
#include <Eigen/Dense>
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
int main(int argc, char **argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
    PointCloudT::Ptr cloud_source_in_x(new PointCloudT());

    pcl::io::loadPCDFile("/home/ahpc/models/alpha/charge_port_alpha_selected_sub.pcd", *cloud_source_in_x);
    Eigen::Matrix4f T;
    Eigen::Matrix4f T_inv;

    // T << 0.930824, 0.0125933, -0.365147, 0.0883422,
    //     0.0184825, 0.996463, 0.0814996, -0.0397862,
    //     0.364898, -0.0826153, 0.927336, 0.247729,
    //     0, 0, 0, 1;
    T << cos(-1.57), -sin(-1.57), 0, 0,
        sin(-1.57), cos(-1.57), 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;
    PointT point;
    
     for (size_t i = 0; i < cloud_source_in_x->size(); i++)
    {
        point.x = cloud_source_in_x->points[i].x / 1000;
        point.y = cloud_source_in_x->points[i].y / 1000;
        point.z = cloud_source_in_x->points[i].z / 1000;
        cloud_in->push_back(point);
    }

    T_inv = T.inverse();
    pcl::transformPointCloud(*cloud_in, *cloud_in, T_inv);
    std::cout << " cloud size-------------" << cloud_in->size() << std::endl;

    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
	voxel_grid.setInputCloud(cloud_in);
	voxel_grid.setLeafSize(0.0008, 0.0008, 0.0008);
	voxel_grid.filter(*cloud_in);

    // pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    // sor.setInputCloud(cloud_in);
    // sor.setMeanK(50);
    // sor.setStddevMulThresh(0.01);
    // sor.setNegative(false);
    // sor.filter(*cloud_in);
    std::cout << "after filter cloud size-------------" << cloud_in->size() << std::endl;
    pcl::io::savePCDFile("/home/ahpc/models/alpha/alpha_models.pcd", *cloud_in);

    // boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_total_points(new pcl::visualization::PCLVisualizer("Tatal_point_Viewer"));

    // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
    //     cloud_tr_color_h(cloud_in, 255, 0, 0);
    // viewer_total_points->addPointCloud(cloud_in, cloud_tr_color_h, "total_points"); //  设置total_points点云为红色

    // // viewer_total_points->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud_raw");
    // // viewer_total_points->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "circle_points");

    // viewer_total_points->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "total_points");
    // // viewer_total_points->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 3, "circle_points");
    // // viewer_total_points->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 3, "plane");

    // viewer_total_points->addCoordinateSystem(0.2);

    // while (!viewer_total_points->wasStopped())
    // {
    //     viewer_total_points->spinOnce(100);
    // }
}