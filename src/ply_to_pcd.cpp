
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>

int main(int argc, char **argv)
{   
    typedef pcl::PointXYZ pointT;
    typedef pcl::PointCloud<pointT> pointcloudtT;

    pointcloudtT::Ptr cloud_in(new pointcloudtT);
    // pcl::io::loadPLYFile<pointT>(argv[1],*cloud_in);
    // pcl::io::savePCDFile("../data/resulttrans_poincloud.pcd", *cloud_in);

    pcl::io::loadPCDFile<pointT>("/home/ahpc/fanglu_workspace/precise_location/test/chongdiankou.pcd",*cloud_in);
    pcl::io::savePLYFile("../data/resulttrans_poincloud.ply", *cloud_in);

    return 0;
}