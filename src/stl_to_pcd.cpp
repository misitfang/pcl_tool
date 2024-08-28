#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>//loadPolygonFileOBJ所属头文件；
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>

using namespace std;
using namespace pcl;
int main()
{

	//读取STL格式模型
	vtkSmartPointer<vtkSTLReader> reader = vtkSmartPointer<vtkSTLReader>::New();
	reader->SetFileName("/home/ahpc/models/alpha/raw/8800227413_005_20230907_alpha_new.stl");
	reader->Update();
	vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
	polydata = reader->GetOutput();
	polydata->GetNumberOfPoints();
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	//从ply转pcd
	pcl::io::vtkPolyDataToPointCloud(polydata, *cloud);
    pcl::PointXYZ point;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_point(new pcl::PointCloud<pcl::PointXYZ>());
    for (size_t i=0;i<cloud->size();i++){
        point.x=cloud->points[i].x/1000;
        point.y=cloud->points[i].y/1000;
        point.z=cloud->points[i].z/1000;
        cloud_point->push_back(point);
    }
	pcl::io::savePCDFileASCII("../data/pcd/model/8800227413_005_20230907_alpha_new.pcd", *cloud_point);
	//PCLVisualizer 显示原STL文件
	pcl::visualization::PCLVisualizer vis;
	vis.addModelFromPolyData(polydata, "mesh", 0);
	//保存pcd文件
	while (!vis.wasStopped())
	{
		vis.spinOnce();
	}
}


