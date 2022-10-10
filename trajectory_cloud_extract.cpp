#include <fstream>
#include <iostream>
#include <pcl/console/parse.h>
#include <pcl/features/boundary.h>
#include <pcl/features/eigen.h>
#include <pcl/features/feature.h>
#include <pcl/features/normal_3d.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/io/pcd_io.h>   //文件输入输出
#include <pcl/point_types.h> //点类型相关定义
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <strstream>
#include <vector>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/common.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

void readTxtFile(const std::string &fileName, const char tag, const PointCloudT::Ptr &pointCloud)
{
    std::cout << "reading file start..... " << std::endl;
    ifstream fin(fileName);
    std::string linestr;
    std::vector<PointT> myPoint;
    while (getline(fin, linestr))
    {
        std::vector<std::string> strvec;
        std::string s;
        std::stringstream ss(linestr);
        while (getline(ss, s, tag))
        {
            strvec.push_back(s);
        }
        if (strvec.size() < 3)
        {
            std::cout << "格式不支持" << std::endl;
        }
        PointT p;
        p.x = stod(strvec[0]);
        p.y = stod(strvec[1]);
        p.z = stod(strvec[2]);
        myPoint.push_back(p);
    }
    fin.close();

    //转换成pcd
    pointCloud->width = (int)myPoint.size();
    pointCloud->height = 1;
    pointCloud->is_dense = false;
    pointCloud->points.resize(pointCloud->width * pointCloud->height);
    for (int i = 0; i < myPoint.size(); i++)
    {
        pointCloud->points[i].x = myPoint[i].x;
        pointCloud->points[i].y = myPoint[i].y;
        pointCloud->points[i].z = myPoint[i].z;
    }
    std::cout << "reading file finished! " << std::endl;
    std::cout << "There are " << pointCloud->points.size() << " points!" << std::endl;
}
int main()
{
    PointCloudT::Ptr cloud_in(new PointCloudT);
    PointCloudT::Ptr cloud_extr_filter(new PointCloudT);

    readTxtFile("/home/lu/Download/pcl_tools/src/data/test.txt", ' ', cloud_in);
    pcl::io::savePCDFile("a.pcd", *cloud_in);
    std::cout << "There are " << cloud_in->points.size() << " points before filter!" << std::endl;

    pcl::PassThrough<PointT> pass(true);
    pass.setInputCloud(cloud_in);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(-295, -150);
    pass.setNegative(false);
    pass.filter(*cloud_in);

    pcl::PassThrough<PointT> pass1(true);
    pass1.setInputCloud(cloud_in);
    pass1.setFilterFieldName("x");
    pass1.setFilterLimits(-300, 154);
    pass1.setNegative(false);
    pass1.filter(*cloud_in);

    // pcl::StatisticalOutlierRemoval<PointT> sor;
    // sor.setInputCloud(cloud_in);
    // sor.setMeanK(50);
    // sor.setStddevMulThresh(1);
    // sor.filter(*cloud_in);

    // pcl::VoxelGrid<PointT> vg;
    // vg.setInputCloud(cloud_in);
    // vg.setLeafSize(0.01,0.01,0.01);
    // vg.filter(*cloud_in);

    pcl::NormalEstimation<PointT, pcl::Normal> ne;
    ne.setInputCloud(cloud_in);
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
    ne.setSearchMethod(tree);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    ne.setRadiusSearch(0.3);
    ne.compute(*cloud_normals);

   
    PointT minpoint, maxpoint;
    pcl::getMinMax3D(*cloud_in, minpoint, maxpoint);
    std::cout << "There are " << cloud_in->points.size() << " points after filter!" << std::endl;
    std::cout << "----------------------------------" << std::endl;
    std::cout << "minpoint.x: " << minpoint.x << std::endl;
    std::cout << "maxpoint.x: " << maxpoint.x << std::endl;
    std::cout << "minpoint.y: " << minpoint.y << std::endl;
    std::cout << "maxpoint.y: " << maxpoint.y << std::endl;
    std::cout << "minpoint.z: " << minpoint.z << std::endl;
    std::cout << "maxpoint.z: " << maxpoint.z << std::endl;

    PointCloudT::Ptr final_cloud(new PointCloudT);
    std::vector<PointCloudT::Ptr> cloud_out; //
    std::vector<PointCloudT::Ptr> cloud_out_1;
    double tmp = (maxpoint.x + minpoint.x) / 2;
    int distances = 7;
    std::cout << "tmp" << tmp << std::endl;
    int a = ceil(maxpoint.x - minpoint.x) / (2 * distances); //向上取整，求在distances下有多少条线
    std::cout << "a: " << a << std::endl;
    for (size_t j = 0; j < a - 4; j++)
    {
        PointCloudT::Ptr tmp_cloud(new PointCloudT);
        for (int i = 1; i < cloud_in->size(); i++)
        {
            if (-0.25 < tmp + distances * j - cloud_in->points[i].x && tmp + distances * j - cloud_in->points[i].x < 0.25 &&
                70 < cloud_in->points[i].y - minpoint.y && maxpoint.y - cloud_in->points[i].y > 70)
            {
                tmp_cloud->points.push_back(cloud_in->points[i]);
            }
        }
        cloud_out.push_back(tmp_cloud); //每条线上的点云到cloud_out中
    }
    for (size_t i = 0; i < cloud_out.size(); i++)
    {
        PointT max, min, tmp_point_min, tmp_point_max;
        pcl::getMinMax3D(*cloud_out[i], min, max); //求取每条线上的y最大最小值
        for (size_t j = 0; j < cloud_out[i]->size(); j++)
        {
            if (min.y == cloud_out[i]->points[j].y)
            {
                tmp_point_min = cloud_out[i]->points[j];
            }
            else if (max.y == cloud_out[i]->points[j].y)
            {
                tmp_point_max = cloud_out[i]->points[j];
            }
        }

        //判断左右两点先后顺序
        if (!(((i + 1) % 2) == 0))
        {
            std::cout << "下半部分第" << 2 * i + 1 << "个点坐标:x " << tmp_point_min.x << " y: "
                      << tmp_point_min.y << " z: " << tmp_point_min.z << std::endl;
            final_cloud->push_back(tmp_point_min);
            std::cout << "下半部分第" << 2 * i + 2 << "个点坐标:x " << tmp_point_max.x << " y: "
                      << tmp_point_max.y << " z: " << tmp_point_max.z << std::endl;
            final_cloud->push_back(tmp_point_max);
        }
        else
        {
            std::cout << "下半部分第" << 2 * i + 1 << "个点坐标:x " << tmp_point_max.x << " y: "
                      << tmp_point_max.y << " z: " << tmp_point_max.z << std::endl;
            final_cloud->push_back(tmp_point_max);
            std::cout << "下半部分第" << 2 * i + 2 << "个点坐标:x " << tmp_point_min.x << " y: "
                      << tmp_point_min.y << " z: " << tmp_point_min.z << std::endl;
            final_cloud->push_back(tmp_point_min);
        }
    }
    pcl::io::savePCDFile("/home/lu/Download/pcl_tools/src/data/finalcloud.pcd", *final_cloud);
    //上半部
    for (size_t j = 0; j < a - 4; j++)
    {
        PointCloudT::Ptr tmp_cloud(new PointCloudT);
        for (int i = 1; i < cloud_in->size(); i++)
        {
            if (-0.25 < tmp - distances * j - cloud_in->points[i].x && tmp - distances * j - cloud_in->points[i].x < 0.25 &&
                70 < cloud_in->points[i].y - minpoint.y && maxpoint.y - cloud_in->points[i].y > 70)
            {
                tmp_cloud->points.push_back(cloud_in->points[i]);
            }
        }
        cloud_out_1.push_back(tmp_cloud); //每条线上的点云到cloud_out_1中
    }
    for (size_t i = 0; i < cloud_out_1.size(); i++)
    {
        PointT max, min, tmp_point_min, tmp_point_max;
        pcl::getMinMax3D(*cloud_out_1[i], min, max); //求取每条线上的y最大最小值
        for (size_t j = 0; j < cloud_out_1[i]->size(); j++)
        {
            if (min.y == cloud_out_1[i]->points[j].y)
            {
                tmp_point_min = cloud_out_1[i]->points[j];
            }
            else if (max.y == cloud_out_1[i]->points[j].y)
            {
                tmp_point_max = cloud_out_1[i]->points[j];
            }
        }
        if (((i + 1) % 2) == 0)
        {
            std::cout << "上半部分第" << 2 * i + 1 << "个点坐标:x " << tmp_point_max.x << " y: " << tmp_point_max.y << " z: " << tmp_point_max.z << std::endl;
            std::cout << "上半部分第" << 2 * i + 2 << "个点坐标:x " << tmp_point_min.x << " y: " << tmp_point_min.y << " z: " << tmp_point_min.z << std::endl;
        }
        else
        {
            std::cout << "上半部分第" << 2 * i + 1 << "个点坐标:x " << tmp_point_min.x << " y: " << tmp_point_min.y << " z: " << tmp_point_min.z << std::endl;
            std::cout << "上半部分第" << 2 * i + 2 << "个点坐标:x " << tmp_point_max.x << " y: " << tmp_point_max.y << " z: " << tmp_point_max.z << std::endl;
        }
    }
    PointCloudT::Ptr cloud_out_viewer(new PointCloudT);

    //下半部分结果可视化所有点求取
    for (size_t i = 0; i < cloud_in->size(); i++)
    {
        for (int j = 1; j < a - 4; j++)
        {
            if (-0.25 < tmp + distances * j - cloud_in->points[i].x && tmp + distances * j - cloud_in->points[i].x < 0.25 && 70 < cloud_in->points[i].y - minpoint.y && maxpoint.y - cloud_in->points[i].y > 70)
            {
                cloud_out_viewer->points.push_back(cloud_in->points[i]);
                //  std::cout << "points" << i << "x: " << cloud_in->points[i].x << " y: " << cloud_in->points[i].y << " z: " << cloud_in->points[i].z << std::endl;
            }
        }
    }

    //上半部分
    for (size_t i = 0; i < cloud_in->size(); i++)
    {
        for (int j = 1; j < a - 4; j++)
        {
            if (-0.25 < tmp - distances * j - cloud_in->points[i].x && tmp - distances * j - cloud_in->points[i].x < 0.25 && 70 < cloud_in->points[i].y - minpoint.y && maxpoint.y - cloud_in->points[i].y > 70)
            {
                cloud_out_viewer->points.push_back(cloud_in->points[i]);
                //  std::cout << "points" << i << "x: " << cloud_in->points[i].x << " y: " << cloud_in->points[i].y << " z: " << cloud_in->points[i].z << std::endl;
            }
        }
    }
    cloud_out_viewer->width = 1;
    cloud_out_viewer->height = cloud_out_viewer->points.size();
    pcl::io::savePCDFile("/home/lu/Download/pcl_tools/src/data/trajectory_cloud.pcd", *cloud_out_viewer);

    std::cout << "cloud_out size is : " << cloud_out.size() << std::endl;
    pcl::visualization::PCLVisualizer viewer;
    // pcl::visualization::PCLVisualizer viewer1("PCL Viewer");
    // viewer1.setBackgroundColor(0.0, 0.0, 0.5);
    // viewer1.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud_in, cloud_normals);
    // viewer1.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud_in");
    // viewer1.addCoordinateSystem(200.0, 0);
    viewer.addCube(minpoint.x, maxpoint.x, minpoint.y, maxpoint.y, minpoint.z, maxpoint.z, 255, 0, 0, "cube");
    viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "cube");
    // viewer1.initCameraParameters();
    // pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> fildColor(cloud_in, "z");z轴渲染
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud_in, 100, 255, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> bule_color(cloud_in, 0, 0, 255);

    //  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud_in);
    viewer.addPointCloud<PointT>(cloud_in, single_color, "cloud_in");
    viewer.addPointCloud<PointT>(cloud_out_viewer, bule_color, "cloud_out");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cloud_out");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud_in");
    // viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud_in, cloud_normals, 1, 40, "normal");法向量
    viewer.addCoordinateSystem(200.0, 0);
    viewer.initCameraParameters();

    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
    }
}
