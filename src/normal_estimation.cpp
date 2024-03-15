#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/console/time.h>   // 控制台计算时间
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_normal_plane.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <Eigen/Core>
#define PI 3.14159265
#define PRAD 180 / PI

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

int main(int argc, char **argv)
{
    double nor_x;
    double nor_y;
    double nor_z;
    double theta;
    Eigen::Vector3d z_axis(0, 0, 1);

    PointCloudT::Ptr cloud(new PointCloudT);
    PointCloudT::Ptr cloud_in(new PointCloudT);

    if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud) == -1)
    {
        PCL_ERROR("cloud not load the pcd file");
        return (-1);
    }
    std::cout<<"size before filter : "<<cloud->size()<<std::endl;
    pcl::StatisticalOutlierRemoval<PointT> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(30);
    sor.setStddevMulThresh(1);
    sor.setNegative(false);
    sor.filter(*cloud);
    std::cout<<"size after filter : "<<cloud->size()<<std::endl;
    pcl::io::savePCDFile("../sat.pcd",*cloud);
    PointT point;
    for (size_t i = 0; i < cloud->size(); i++)
    {
        point.x = cloud->points[i].x / 1000;
        point.y = cloud->points[i].y / 1000;
        point.z = cloud->points[i].z / 1000;
        cloud_in->push_back(point);
    }
    pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_plane(new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(cloud));
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_plane);
    ransac.setDistanceThreshold(0.001);
    ransac.setMaxIterations(1000);
    // ransac.setProbability(0.99);
    ransac.computeModel();
    Eigen::VectorXf coeff;
    ransac.getModelCoefficients(coeff);
	pcl::console::TicToc time,time1;
	time.tic();

    // std::cout<<"平面模型系数参数为： "<<coeff[0]<<" \t"<<coeff[1]<<" \t"<<coeff[2]<<" \t"<<coeff[3]<<std::endl;
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    // inliers表示误差能容忍的点 记录的是点云的序号
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    // 创建分割器
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.0018);
    seg.setMaxIterations(2000);
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);
    pcl::ModelCoefficients plane_coeff;
    plane_coeff.values.resize(4);
    plane_coeff.values[0] = coefficients->values[0];
    plane_coeff.values[1] = coefficients->values[1];
    plane_coeff.values[2] = coefficients->values[2];
    plane_coeff.values[3] = coefficients->values[3];
    std::cout << "平面拟合耗时: " << time.toc() << " ms" << std::endl;

    std::cout<<"内点个数为: "<<inliers->indices.size()<<std::endl;
    std::cerr << "Plane Model coefficients: " << coefficients->values[0] << " "
              << coefficients->values[1] << " "
              << coefficients->values[2] << " "
              << coefficients->values[3] << std::endl;
    Eigen::Vector3d plane_result(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
    plane_result.normalized();
    double theta_1,theta2;
    double norm_2=sqrt(coeff[0]*coeff[0]+coeff[1]*coeff[1]+coeff[2]*coeff[2]);
    double norm_1 = sqrt(coefficients->values[0] * coefficients->values[0] + coefficients->values[1] * coefficients->values[1] +
                         coefficients->values[2] * coefficients->values[2]);
    // std::cout<<"plane_result.normalized: "<<plane_result<<std::endl;
    double norm_plane_result=sqrt(plane_result(0)*plane_result(0)+plane_result(1)*plane_result(1)+plane_result(2)*plane_result(2));
    theta_1 = acos(coefficients->values[2] / (1 * norm_1));
    // std::cout<<"norm_plane_result: "<<norm_plane_result<<std::endl;

    theta2=acos(coeff[2]/norm_2);
    double theta_norm_plane_result=acos(plane_result(2)/norm_plane_result);
    // std::cout << "zz_axis.dot(nor_result) " << z_axis.dot(plane_result) << std::endl;
    // std::cout << "theta of z_axis and nor vector from 平面模型系数参 is : " << theta2 * PRAD << std::endl;
    // std::cout << "theta of z_axis and nor vector from 归一化 is : " << theta_norm_plane_result  << std::endl;

    std::cout << "theta of z_axis and nor vector from 平面分割 is : " << theta_1 * PRAD << std::endl;



    time1.tic();
    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(0.001,0.001,0.001);
    vg.filter(*cloud);
    std::cout<<"size before vg filter : "<<cloud->size()<<std::endl;

    pcl::NormalEstimation<PointT, pcl::Normal> ne; // create nnormal estimation object
    ne.setInputCloud(cloud);
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>); // cretae a empty kdtree
    ne.setSearchMethod(tree);                                               // 传递空的kdtree给　ｎｅ

    pcl::PointCloud<pcl::Normal>::Ptr cloud_normal(new pcl::PointCloud<pcl::Normal>);
    ne.setRadiusSearch(0.5); // 设置半径领域的大小
    ne.setViewPoint(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
    // ne.setKSearch(10);//设置ｋ临近点的个数　与半径领域二选一即可
    ne.compute(*cloud_normal); // 执行法线估计
    bool show_nor = false;
    if (show_nor)
    {
        pcl::visualization::PCLVisualizer viewer("PCL Viewer");
        viewer.addPointCloud<PointT>(cloud, "original_cloud");
        viewer.addPlane(plane_coeff,"plane");
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 0.5, "original_cloud");
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "original_cloud");
        // viewer.addPointCloud(cloud_in);
        viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 4, "plane");
        viewer.addCoordinateSystem(1);
        // viewer.setBackgroundColor(0, 0, 0);
        // 显示点云与法线，4和0.5可以调整法线的疏密与长短
        // viewer.addPointCloudNormals<PointT, pcl::Normal>(cloud_in, cloud_normal, 4, 0.05, "normal");
        while (!viewer.wasStopped())
        {
            viewer.spinOnce(300);
        }
    }
    double nor_x_tmp, nor_y_tmp, nor_z_tmp;
    int nor_size = 0;
    for (size_t i = 0; i < cloud_normal->size(); i++)
    {
        //  std::cout << "normal_x:" << cloud_normal->points[i].normal_x << "  normal_y:" << cloud_normal->points[i].normal_y << "   normal_z:" << cloud_normal->points[i].normal_z << std::endl;
        //  std::cout<< "!pcl_isfinite(cloud_normal->points[i].normal_x):"<<!pcl_isfinite(cloud_normal->points[i].normal_x)<<std::endl;
        if (!isnormal(cloud_normal->points[i].normal_x) ||
            !isnormal(cloud_normal->points[i].normal_y) ||
            !isnormal(cloud_normal->points[i].normal_z))
        {

            continue;
        }
        nor_x_tmp += cloud_normal->points[i].normal_x;
        nor_y_tmp += cloud_normal->points[i].normal_y;
        nor_z_tmp += cloud_normal->points[i].normal_z;
        nor_size++;
    }

    nor_x_tmp /= nor_size;
    nor_y_tmp /= nor_size;
    nor_z_tmp /= nor_size;

    std::cout << "nor_size:" << nor_size << std::endl;
    std::cout << "Average of nor_x: " << nor_x_tmp << " nor_y: " << nor_y_tmp << " nor_z: " << nor_z_tmp << std::endl;
    int nor_size_1 = 0;
    for (size_t i = 0; i < cloud_normal->size(); i++)
    {
        double Average_nor_norm = sqrt(nor_x_tmp * nor_x_tmp + nor_y_tmp * nor_y_tmp + nor_z_tmp * nor_z_tmp);
        double cloud_normal_norm = sqrt(pow(cloud_normal->points[i].normal_x, 2) + pow(cloud_normal->points[i].normal_y, 2) +
                                        pow(cloud_normal->points[i].normal_z, 2));
        double normal_theta = acos((nor_x_tmp * cloud_normal->points[i].normal_x + nor_y_tmp * cloud_normal->points[i].normal_y +
                                    nor_z_tmp * cloud_normal->points[i].normal_z) /
                                   (Average_nor_norm * cloud_normal_norm));
        if (!isnormal(cloud_normal->points[i].normal_x) ||
            !isnormal(cloud_normal->points[i].normal_y) ||
            !isnormal(cloud_normal->points[i].normal_z))
        {
            continue;
        }
        else
        {
            if (normal_theta * 180 / 3.1415 > 30)
            {
                continue;
            }
            nor_x += cloud_normal->points[i].normal_x;
            ;
            nor_y += cloud_normal->points[i].normal_y;
            ;
            nor_z += cloud_normal->points[i].normal_z;
            ;
            nor_size_1++;
        }
    }
    nor_x /= nor_size_1;
    nor_y /= nor_size_1;
    nor_z /= nor_size_1;

    double norm = sqrt(nor_x * nor_x + nor_y * nor_y + nor_z * nor_z);
    std::cout << "平均法向量计算耗时: " << time1.toc() << " ms" << std::endl;

    std::cout << "norm:" << norm << endl;
    theta = acos(nor_z / norm);
    // theta *=PRAD;
    std::cout << "nor_x_result: " << nor_x << " nor_y_result: " << nor_y << " nor_z_result: " << nor_z << std::endl;

    // Eigen::Vector3d nor_result(nor_x, nor_y, nor_z);
    // // std::cout << "nor_result " << z_axis.dot(nor_result) << std::endl;

    // theta = acos(z_axis.dot(nor_result) / (z_axis.norm() * nor_result.norm()));
    // std::cout << "zz_axis.dot(nor_result) " << z_axis.dot(nor_result) << std::endl;

    // std::cout << "z_axis norm " << z_axis.norm() << std::endl;
    std::cout << "theta of z_axis and nor vector from normal estimation is : " << theta * PRAD << std::endl;
}