#include <iostream>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <glog/logging.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

void GolgInit(std::string &log_info_path)
{
    std::string log_info;
    DIR *dir;
    std::string str_command;
    if ((dir = opendir(log_info_path.c_str())) == NULL) {
        LOG(INFO) << "日志存放目录不存在，创建日志存放目录文件" << std::endl;
        str_command = "mkdir -p " + log_info_path;
        system(str_command.c_str()); // 存在就删除目录
    }

    log_info = log_info_path + "charger_process_";
    FLAGS_alsologtostderr = true;
    FLAGS_colorlogtostderr = true;
    google::SetLogDestination(google::GLOG_INFO, log_info.c_str());
    google::SetLogFilenameExtension(".log");

    google::InitGoogleLogging("init logs------");
    LOG(INFO) << "google::IsGoogleLoggingInitialized :" << google::IsGoogleLoggingInitialized() << std::endl;
}

int main(int argc, char **argv)
{
    std::string log_info_path = "../data/logs/charger_process/";
    GolgInit(log_info_path);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
    PointCloudT::Ptr cloud_source_in_x(new PointCloudT());
    PointCloudT::Ptr cloud_out(new PointCloudT());
    PointCloudT::Ptr cloud_filter(new PointCloudT());
    PointCloudT::Ptr cloud_filter_sor(new PointCloudT());

    pcl::io::loadPCDFile("../data/pcd/GB_charger/chongdiankou_eletre_sub_center_open3d.pcd", *cloud_source_in_x);

    PointT point;
    
    for (size_t i = 0; i < cloud_source_in_x->size(); i++)
    {
        point.x = cloud_source_in_x->points[i].x / 1000;
        point.y = cloud_source_in_x->points[i].y / 1000;
        point.z = cloud_source_in_x->points[i].z / 1000;
        cloud_in->push_back(point);
    }
    LOG(INFO) << " cloud in size = " << cloud_in->size() << std::endl;
    
    Eigen::Matrix4f T;
    //坐标系移动到中心位置
    T << 1, 0.0, 0.0, 0.00025,
        0.0, 1, 0.0, -0.0001,
        0.0, 0.0, 1, 0,
        0, 0, 0, 1;
  
    pcl::transformPointCloud(*cloud_in, *cloud_in, T);

  


    for(size_t i = 0; i < cloud_in->size(); i++){

        if(cloud_in->points[i].z > 0.048){
            //剔除底部点云

        }

        //圆环0
        else if ((sqrt(pow(cloud_in->points[i].x-0,2)+pow(cloud_in->points[i].y-0,2)) > 0.0338 || cloud_in->points[i].x>0.03)
        && cloud_in->points[i].z > 0.0008 ){
            //剔除圆环外部点云,保留表面点云z<0.048 圆心 半径(0.0,0.0,0.0)
            // LOG(INFO)<<"圆环0 cloud_in->points[i].xyz " << cloud_in->points[i].x <<","<<cloud_in->points[i].y 
            // <<","<<cloud_in->points[i].z <<std::endl;
                
        }

        //圆环1
        else if (sqrt(pow(cloud_in->points[i].x -0, 2) + pow(cloud_in->points[i].y + 0.017, 2)) < 0.012 && 
            cloud_in->points[i].z > 0.0095 && cloud_in->points[i].z < 0.048){
            //剔除圆环1内部点云,保留表面点云z<0.009 圆心 半径(0.0,-0.017,0.0123)
            // LOG(INFO)<<"圆环1 cloud_in->points[i].xyz " << cloud_in->points[i].x <<","<<cloud_in->points[i].y 
            // <<","<<cloud_in->points[i].z <<std::endl;
        }

        //圆环2
        else if (sqrt(pow(cloud_in->points[i].x -0, 2) + pow(cloud_in->points[i].y - 0.017, 2)) < 0.012 && 
            cloud_in->points[i].z > 0.0095){
            //剔除圆环1内部点云,保留表面点云z<0.009 圆心 半径(0.0,0.017,0.0123)
            // LOG(INFO)<<"圆环2 cloud_in->points[i].xyz " << cloud_in->points[i].x <<","<<cloud_in->points[i].y 
            // <<","<<cloud_in->points[i].z <<std::endl;
        }

        //圆环3
        else if (sqrt(pow(cloud_in->points[i].x + 0.0198, 2) + pow(cloud_in->points[i].y -0, 2)) < 0.007 && 
            cloud_in->points[i].z > 0.009){
            //剔除圆环3内部点云,保留表面点云z<0.009 圆心 半径(-0.0198,0,0.00725)
            // LOG(INFO)<<"圆环3 cloud_in->points[i].xyz " << cloud_in->points[i].x <<","<<cloud_in->points[i].y 
            // <<","<<cloud_in->points[i].z <<std::endl;
        }

        //圆环4
        else if(sqrt(pow(cloud_in->points[i].x + 0.0198, 2) + pow(cloud_in->points[i].y + 0.01425, 2)) < 0.004&& 
            cloud_in->points[i].z >0.0185){
            //剔除圆环4。 圆环4圆心 半径(-0.0198,-0.01425,0.0045) 
        }

        //圆环5
        else if (sqrt(pow(cloud_in->points[i].x + 0.0198, 2) + pow(cloud_in->points[i].y - 0.01425, 2)) < 0.004&& 
            cloud_in->points[i].z >0.0185)
        {
            // LOG(INFO)<<"圆环5 cloud_in->points[i].z " << cloud_in->points[i].z <<std::endl;
           //剔除圆环5。 圆环5圆心 半径(-0.0198,0.01425,0.0045)
        }


        //圆环6
        else if (sqrt(pow(cloud_in->points[i].x - 0.0102, 2) + pow(cloud_in->points[i].y - 0.0, 2)) < 0.004 && 
            cloud_in->points[i].z >0.0185)
        {
            //剔除圆环6内部点云。 圆环6圆心 半径(0.0102,0.0,0.0045)
            // LOG(INFO)<<"圆环6 cloud_in->points[i].xyz " << cloud_in->points[i].x <<","<<cloud_in->points[i].y 
            // <<","<<cloud_in->points[i].z <<std::endl;
        }

        //圆环7。
        else if (sqrt(pow(cloud_in->points[i].x - 0.0222, 2) + pow(cloud_in->points[i].y + 0.012, 2)) < 0.004&& 
            cloud_in->points[i].z >0.0185)
        {
            //剔除圆环7。圆环7圆心 半径(0.0222,-0.012,0.0045)
        }

        //圆环8
        else if (sqrt(pow(cloud_in->points[i].x - 0.0222, 2) + pow(cloud_in->points[i].y - 0.0, 2)) < 0.004 && 
            cloud_in->points[i].z > 0.0095)
        {
            //剔除圆环6内部点云。 圆环6圆心 半径(0.0222,0.0,0.0045)
            // LOG(INFO)<<"圆环8 cloud_in->points[i].xyz " << cloud_in->points[i].x <<","<<cloud_in->points[i].y 
            // <<","<<cloud_in->points[i].z <<std::endl;
        }

        //圆环9。
        else if (sqrt(pow(cloud_in->points[i].x - 0.0222, 2) + pow(cloud_in->points[i].y - 0.012, 2)) < 0.004&& 
            cloud_in->points[i].z >0.0185)
        {
            //剔除圆环9。 圆环9圆心 半径(0.0222,0.012,0.0045)
        }

        else if(sqrt(pow(cloud_in->points[i].x-0,2)+pow(cloud_in->points[i].y-0,2)) > 0.033 && cloud_in->points[i].x < 0 &&
        cloud_in->points[i].z > 0.0008 )
        {
            //杂点
        }
        else
        {
            cloud_out->push_back(cloud_in->points[i]);/* code */
        }
            
    }
    LOG(INFO) << " cloud out size = " << cloud_out->size() << std::endl;

    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
	voxel_grid.setInputCloud(cloud_out);
	voxel_grid.setLeafSize(0.0007, 0.0007, 0.0007);
	voxel_grid.filter(*cloud_filter);

    // pcl::ApproximateVoxelGrid<PointT> avg;
    // avg.setInputCloud(cloud_out);
	// avg.setLeafSize(0.0001, 0.0001, 0.0001);
	// avg.filter(*cloud_filter);
    LOG(INFO) << " cloud out size after VG filter  = " << cloud_filter->size() << std::endl;

    // // 统计滤波器
    // pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    // sor.setInputCloud(cloud_filter);
    // sor.setMeanK(1100);
    // sor.setStddevMulThresh(1);
    // sor.setNegative(false);
    // sor.filter(*cloud_filter_sor);
    // LOG(INFO) << " cloud out size after sor filter  = " << cloud_filter_sor->size() << std::endl;

    
    pcl::io::savePCDFile("../data/result/charger_process/charger_out.pcd", *cloud_out);
    pcl::io::savePCDFile("../data/result/charger_process/charger_out_filter.pcd", *cloud_filter);

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_total_points(new pcl::visualization::PCLVisualizer("Tatal_point_Viewer"));

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_in_color_(cloud_in, 255, 0, 0);
    // viewer_total_points->addPointCloud(cloud_in, cloud_in_color_, "cloud_in"); //  设置cloud_in点云为红色

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_out_color_(cloud_filter, 0, 255, 0);
    viewer_total_points->addPointCloud(cloud_filter, cloud_out_color_, "cloud_out"); //  设置cloud_out点云为绿色

    // viewer_total_points->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud_raw");
    // viewer_total_points->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "circle_points");

    // viewer_total_points->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 3, "cloud_in");
    viewer_total_points->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud_in");
    viewer_total_points->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud_out");
    // viewer_total_points->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 3, "cloud_out");

    viewer_total_points->addCoordinateSystem(0.03);

    while (!viewer_total_points->wasStopped())
    {
        viewer_total_points->spinOnce(100);
    }
}