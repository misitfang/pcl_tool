#include <iostream>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/filter_indices.h> // for pcl::removeNaNFromPointCloud
#include <pcl/segmentation/region_growing.h>
#include <pcl/filters/statistical_outlier_removal.h>
int main()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/ahpc/myspace/pcl_tool/pcl_tool/data/loam/finalCloud.pcd", *cloud) == -1)
    {
        std::cout << "Cloud reading failed." << std::endl;
        return (-1);
    }

    pcl::search::Search<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
    normal_estimator.setSearchMethod(tree);
    normal_estimator.setInputCloud(cloud);
    normal_estimator.setKSearch(50);
    normal_estimator.compute(*normals);
    pcl::IndicesPtr indices(new std::vector<int>);
    pcl::removeNaNFromPointCloud(*cloud, *indices);
    pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
    reg.setMinClusterSize(50);
    reg.setMaxClusterSize(1000000);
    reg.setSearchMethod(tree);
    reg.setNumberOfNeighbours(30);
    reg.setInputCloud(cloud);
    reg.setIndices(indices);
    reg.setInputNormals(normals);
    reg.setSmoothnessThreshold(6.0 / 180.0 * M_PI);
    reg.setCurvatureThreshold(2.0);

    std::vector<pcl::PointIndices> clusters;
    reg.extract(clusters);

    std::cout << "Number of clusters is equal to " << clusters.size() << std::endl;
    for (size_t i = 0; i < clusters.size(); i++)
    {
        std::cout << "clusters[" << i << "] has " << clusters[i].indices.size() << " points " << std::endl;
    }
    std::cout << "First cluster has " << clusters[0].indices.size() << " points." << std::endl;
    std::cout << "These are the indices of the points of the initial" << std::endl
              << "cloud that belong to the first cluster:" << std::endl;
    std::size_t counter = 0;
    while (counter < clusters[4].indices.size())
    {
        std::cout << clusters[0].indices[counter] << ", ";
        cloud->points[clusters[0].indices[counter]].x = 0.0;
        cloud->points[clusters[0].indices[counter]].y = 0.0;
        cloud->points[clusters[0].indices[counter]].z = 0.0;
        counter++;
        if (counter % 10 == 0)
            std::cout << std::endl;
    }
    std::cout << std::endl;
    std::cout<<"size of cloud is "<<cloud->size()<<std::endl;

    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(50);
    sor.setStddevMulThresh(1);
    sor.filter(*cloud);
    std::cout<<"size of cloud is "<<cloud->size()<<std::endl;
    normal_estimator.setSearchMethod(tree);
    normal_estimator.setInputCloud(cloud);
    normal_estimator.setKSearch(50);
    normal_estimator.compute(*normals);
    pcl::IndicesPtr indices1(new std::vector<int>);
    pcl::removeNaNFromPointCloud(*cloud, *indices);
    // pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg1;
    reg.setMinClusterSize(50);
    reg.setMaxClusterSize(1000000);
    reg.setSearchMethod(tree);
    reg.setNumberOfNeighbours(330);
    reg.setInputCloud(cloud);
    reg.setIndices(indices1);
    reg.setInputNormals(normals);
    reg.setSmoothnessThreshold(30.0 / 180.0 * M_PI);
    reg.setCurvatureThreshold(1.0);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud();
    pcl::visualization::CloudViewer viewer("Cluster viewer");
    viewer.showCloud(colored_cloud);
    while (!viewer.wasStopped())
    {
    }
    return (0);
}