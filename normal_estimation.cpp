#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <Eigen/Core>
#define PI 3.14159265
#define PRAD 180/PI
int main(int argc, char **argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/lu/Workspace/PCL/pcl_sources/a_feature/selected.pcd", *cloud) == -1)
    {
        PCL_ERROR("cloud not load the pcd file");
        return (-1);
    }
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne; // create nnormal estimation object

    ne.setInputCloud(cloud);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>); // cretae a empty kdtree
    ne.setSearchMethod(tree);                                                             //传递空的kdtree给　ｎｅ

    pcl::PointCloud<pcl::Normal>::Ptr cloud_normal(new pcl::PointCloud<pcl::Normal>);
    ne.setRadiusSearch(0.5); //设置半径领域的大小
    ne.setViewPoint (std::numeric_limits<float>::max (), std::numeric_limits<float>::max (), std::numeric_limits<float>::max ());
    // ne.setKSearch(10);//设置ｋ临近点的个数　与半径领域二选一即可
    ne.compute(*cloud_normal); //执行法线估计

    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    viewer.addPointCloud<pcl::PointXYZ>(cloud, "original cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 0.7, "original cloud");

    // viewer.setBackgroundColor(0.0, 0.0, 0.5);
    viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, cloud_normal);
    pcl::io::savePCDFile("normal_estimation.pcd", *cloud_normal);
    double x = 0;
    double y = 0;
    double z = 0;
    int j = 0;
    for (size_t i = 0; i < cloud_normal->size(); i++)
    {
        std::cout << "normal_x:" << cloud_normal->points[i].normal_x << "  normal_y:" << cloud_normal->points[i].normal_y << "   normal_z:" << cloud_normal->points[i].normal_z << std::endl;
        if (!pcl_isfinite(cloud_normal->points[i].normal_x)||
        !pcl_isfinite(cloud_normal->points[i].normal_y)||
        !pcl_isfinite(cloud_normal->points[i].normal_z))
        {
            continue;
        }
        x += cloud_normal->points[i].normal_x;
        y += cloud_normal->points[i].normal_y;
        z += cloud_normal->points[i].normal_z;

        std::cout << "for x:" << x << endl;
        j++;
        // std::cout<<" curvature:"<<cloud_normal->points[i].curvature<<std::endl;
    }
    x /= j;
    y /= j;
    z /= j;

   
    
    std::cout << "x y z:" << x  << y << z << endl;
    double norm =x * x + y * y + z * z;
    std::cout << "norm:" << x * x + y * y + z * z << endl;
    double theta=acos(z);
    theta *=PRAD;
    std::cout<<"theta:"<<theta<<std::endl;
    Eigen::Vector3d q;
    q << y, -x, 0;
    q.normalize();

    std::cout << "cross:" << q.transpose()<< std::endl;

    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
    }
}