#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
// #include<opencv2/opencv.hpp>
#include <fstream>
 
 
// using namespace cv;
using namespace std;
 
 
//分割字符串的方法
void split(std::string& string_input, std::vector<std::string>&string_output, std::string& delema1)
{
	std::string::size_type start = string_input.find_first_not_of(delema1, 0);//找到第一个不为delema1的下标
	std::string::size_type pose = string_input.find_first_of(delema1, start);//找到第一个delema1的下标
	while (std::string::npos != start || std::string::npos != pose) {//当即没有delema1也没有字符的时候结束
		string_output.push_back(string_input.substr(start, pose - start));
		start = string_input.find_first_not_of(delema1, pose);//更新start 从pose开始
		pose = string_input.find_first_of(delema1, start);//更新pose,从start开始再次寻找
	}
}
 
 
//加载点云
bool get_parameter_xyz(std::string path, pcl::PointCloud<pcl::PointXYZ> &cloud,std::vector<pcl::PointXYZ> &vector3d)
{
	cloud.clear();
	std::ifstream inf;
	bool flag = false;
	try
	{
		inf.open(path);
	}
	catch (const std::exception&)
	{
		
		return false;
	}
	std::string sline;//每一行
	std::vector<std::string>string_output;
	std::string Delema = " ";
	pcl::PointXYZ point3d;
    // std::vector<pcl::PointXYZ> vector3d;
	while (getline(inf, sline))
	{
		int i = 0;
		split(sline, string_output, Delema);
		point3d.x = stold(string_output[i]);
		point3d.y = stold(string_output[i+1]);
		point3d.z = stold(string_output[i + 2]);
		string_output.clear();
		cloud.push_back(point3d);
        vector3d.push_back(point3d);
	}
    // for (int i=0;i<vector3d.size();i++){
    //     std::cout<<"point3d x:"<<vector3d[i].x<<std::endl;

    // }
	return true;
	inf.close();
}
 
int main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<pcl::PointXYZ> vector3d;
	get_parameter_xyz(argv[1], *cloud,vector3d);
    std::cout<<
    pcl::io::savePCDFile(argv[2],*cloud);

	std::cout << "Loaded "
		<< cloud->width * cloud->height
		<< " data points from test_pcd.pcd with the following fields: "
		<< std::endl;
	return 0;
}
