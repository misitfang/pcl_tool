#include <fstream>
#include <iostream>
#include <pcl/io/pcd_io.h>   //文件输入输出
#include <pcl/point_types.h> //点类型相关定义
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <strstream>
#include <vector>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

void readTxtFile(const std::string &fileName, const char tag, const PointCloudT::Ptr &pointCloud)
{
    std::cout << "reading file start..... " << std::endl;
    ifstream fin(fileName);
    std::string linestr;
    std::vector<PointT> myPoint;
    while (getline(fin, linestr)) {
        std::vector<std::string> strvec;
        std::string s;
        std::stringstream ss(linestr);
        while (getline(ss, s, tag)) {
            strvec.push_back(s);
        }
        if (strvec.size() < 3) {
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
    for (int i = 0; i < myPoint.size(); i++) {
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
    
    readTxtFile("/home/flu/workspace/pcl_tools/src/data/test.txt", ' ', cloud_in);
    pcl::io::savePCDFile("a.pcd", *cloud_in);


    pcl::PassThrough<PointT> pass(true);
    pass.setInputCloud(cloud_in);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(-300,-150);
    pass.setNegative(false);
    pass.filter(*cloud_in);
    // std::vector<int> indices_z;

    // pass.filter(indices_z);

    // pcl::PointIndices indices_z_pd = std::make_shared<std::vector<int>>(indices_z);
    
    // pcl::ExtractIndices<PointT> ext;
    // ext.setInputCloud(cloud_in);
    // ext.setIndices(std::make_shared< pcl::PointIndices>(indices_z));
    // ext.filter(cloud_extr_filter);
    

    pcl::visualization::PCLVisualizer viewer;
     pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud_in, 100, 255, 0);
    //  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud_in);
    viewer.addPointCloud<PointT>(cloud_in,single_color, "cloud_in");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud_in");
    viewer.addCoordinateSystem(200.0,0);
    viewer.initCameraParameters ();  





    
    while (!viewer.wasStopped()) {
        viewer.spinOnce();
    }
}
