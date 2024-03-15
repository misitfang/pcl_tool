#include <pcl/registration/icp.h>
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
// #include <pcl/kdtree/flann.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

void icp_test(const PointCloudT::Ptr &points_model,
              PointCloudT::Ptr &points_cloud, const int &iters, const double &epsilon,
              const double &min_err, Eigen::Matrix3d &R, Eigen::Vector3d &t)
{
    // default settings.
    const double min_err2 = min_err * min_err;
    const double factor = 9.0;
    const int n_selected_pts = 100;
    const int step = points_cloud->size() / n_selected_pts; // step for select points.
    std::vector<Eigen::Vector3d> pts_model;//模版点云vector
    for (size_t i = 0; i < points_model->size(); i++)
    {
        Eigen::Vector3d pts_point_tmp;
        pts_point_tmp[0] = points_model->points[i].x;
        pts_point_tmp[1] = points_model->points[i].y;
        pts_point_tmp[2] = points_model->points[i].z;
        pts_model.push_back(pts_point_tmp);
    }
    // two vectors for matched points.
    std::vector<Eigen::Vector3d> pts_cloud_matched;
    pts_cloud_matched.reserve(n_selected_pts);
    std::vector<Eigen::Vector3d> pts_model_matched;
    pts_model_matched.reserve(n_selected_pts);

    // construct kd-tree for model cloud.
    PointCloudT::Ptr model_cloud(new PointCloudT());
    PointT point;
    for (size_t i = 0; i < points_model->size(); i++)
    {
        // model_cloud->points[i]=points_model->points[i];
        point.x = points_model->points[i].x;
        point.y = points_model->points[i].y;
        point.z = points_model->points[i].z;
        model_cloud->push_back(point);
    }
    pcl::KdTreeFLANN<pcl::PointXYZ> *kd_tree = new pcl::KdTreeFLANN<pcl::PointXYZ>();
    kd_tree->setInputCloud(model_cloud);

    // used for search.
    std::vector<int> index(1);
    std::vector<float> squared_distance(1);

    // Dth
    double squared_distance_th = std::numeric_limits<double>::max();
    double cur_squared_dist = 0.0;
    double last_squared_dist = std::numeric_limits<double>::max();

    // for n_iters
    for (int n = 0; n < iters; n++)
    {

        // clear two point clouds.
        pts_cloud_matched.clear();
        pts_model_matched.clear();

        // step 1. construct matched point clouds.
        double sum_squared_dist = 0.0;

        for (size_t i = 0; i < points_cloud->size(); i += step)
        {

            // transformed by T
            Eigen::Vector3d pts_cloud;
            pts_cloud[0] = points_cloud->points[i].x;
            pts_cloud[1] = points_cloud->points[i].y;
            pts_cloud[2] = points_cloud->points[i].z;

            Eigen::Vector3d pt = R * pts_cloud + t;

            // find the nearest pints by knn
            pcl::PointXYZ pt_d(pt[0], pt[1], pt[2]);
            if (!kd_tree->nearestKSearch(pt_d, 1, index, squared_distance))
            {
                std::cerr << "ERROR: no points found.\n";
                return;
            }

            if (squared_distance[0] < squared_distance_th)
            {
                // add squared distance.
                sum_squared_dist += squared_distance[0];
                // add the pt in cloud.
                pts_cloud_matched.push_back(pts_cloud);
                // add the pt in model.
                pts_model_matched.push_back(pts_model.at(index[0]));
            }
        } // for all pts_cloud

        // std::cout << "iter:" << n << " mathced size: " << pts_model_matched.size() << " " << pts_cloud_matched.size() << std::endl;

        // step 2. Get R and t.
        // step 2.1 find centor of model(X) and cloud(P)
        Eigen::Vector3d mu_x(0.0, 0.0, 0.0);
        Eigen::Vector3d mu_p(0.0, 0.0, 0.0);
        for (size_t i = 0; i < pts_cloud_matched.size(); i++)
        {
            mu_x += pts_model_matched.at(i);
            mu_p += pts_cloud_matched.at(i);
        }
        mu_x = mu_x / double(
            pts_model_matched.size());
        mu_p = mu_p / double(pts_cloud_matched.size());

        // step 2.2 Get W.

        Eigen::Matrix3d W;
        for (size_t i = 0; i < pts_cloud_matched.size(); i++)
        {
            W += (pts_model_matched.at(i) - mu_x) * ((pts_cloud_matched.at(i) - mu_p).transpose());
        }

        // step 2.3 Get R
        Eigen::JacobiSVD<Eigen::Matrix3d> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);

        R = svd.matrixU() * (svd.matrixV().transpose());

        // step 2.4 Get t
        t = mu_x - R * mu_p;
        std::cout << "R: \n"
                  << R << std::endl;
        std::cout << "t: \n"
                  << t << std::endl;
        // step 3. Check if convergenced.
        cur_squared_dist = sum_squared_dist / (double)pts_cloud_matched.size();
        double squared_dist_change = last_squared_dist - cur_squared_dist;

        // std::cout << "iter:" << n << " squared_dist_change: " << squared_dist_change << " cur distance " << cur_squared_dist  << std::endl;

        if (squared_dist_change < epsilon || cur_squared_dist < min_err2)
        {
            break;
        }
        last_squared_dist = cur_squared_dist;
        squared_distance_th = factor * cur_squared_dist;

    } // for n_iters

    delete kd_tree;
}

int main(int argc, char **argv)
{

    PointCloudT::Ptr cloud_source(new PointCloudT());
    PointCloudT::Ptr cloud_source_in_x(new PointCloudT());

    PointCloudT::Ptr cloud_target(new PointCloudT());
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    Eigen::Vector3d t(0.0, 0.0, 0.0);
    pcl::io::loadPCDFile("../data/icp_process/eletre.pcd", *cloud_source);
    pcl::io::loadPCDFile("../data/icp_process/chargePortCLoud.pcd", *cloud_target);
    icp_test(cloud_source, cloud_target, 50, 1.0e-10, 1.0e-10, R, t);
    std::cout << "R: \n"
              << R << std::endl;
    std::cout << "t: \n"
              << t << std::endl;

    PointT point;
    // pcl::io::loadPCDFile("/home/ahpc/myspace/pcl_tool/pcl_tool/models/pcl_G9_model.pcd", *cloud_source);
    std::cout << "目标点云有 " << cloud_source->size() << " 个点" << endl;
    // for (size_t i = 0; i < cloud_source->size(); i++)
    // {
    //     cloud_source->points[i].x /= 1000;
    //     cloud_source->points[i].y /= 1000;
    //     cloud_source->points[i].z /= 1000;
    // }
    Eigen::Matrix4f T;

    // pcl::StatisticalOutlierRemoval<PointT> sor;
    // sor.setInputCloud(cloud_source);
    // sor.setMeanK(40);
    // sor.setStddevMulThresh(1);
    // sor.setNegative(false);
    // sor.filter(*cloud_source);
    // std::cout << "目标点云滤波后还剩 " << cloud_source->size() << " 个点" << endl;
    // pcl::io::savePCDFile("/home/ahpc/myspace/pcl_tool/pcl_tool/GB_charger_zeek_filter.pcd", *cloud_source);

    //  for (size_t i = 0; i < cloud_source->size(); i++)
    //         {
    //             // 去除充电口上方距离点（0.0283，0.018）平面距离大于0.037m的点
    //             if (cloud_source->points[i].x > 0.025 &&
    //                 sqrt(pow(cloud_source->points[i].x - 0.0283, 2) + pow(cloud_source->points[i].y - 0.018, 2)) > 0.037)
    //             {
    //             }
    //             // 去除充电口下方距离点（0.0，0.017）平面距离大于0.039m的点
    //             else if (cloud_source->points[i].x <= 0.005 &&
    //                      sqrt(pow(cloud_source->points[i].x - 0.0, 2) + pow(cloud_source->points[i].y - 0.017, 2)) > 0.039)
    //             {
    //             }
    //             else if (cloud_source->points[i].z > 0.01)
    //             {
    //                 /* code */
    //             }

    //             else
    //             {
    //                 cloud_source_in_x->push_back(cloud_source->points[i]);
    //             }
    //         }
    // pcl::io::savePCDFile("./filter_zeek_ty.pcd",*cloud_source_in_x);
    // STLXUANZHUAN
    T << 0, 1, 0.0, -0.00398653,
        -1, 0.00, 0.0, -0.0391091,
        0.0, 0.00, 1, 0.359212,
        0, 0, 0, 1;
    // pcl::transformPointCloud(*cloud_source, *cloud_source, T);

    Eigen::Matrix4f T1;

    T1 << 0.99996, -0.000885609, -0.00896047, 0.0696846,
        0.000917621, 0.999995, 0.00357133, -0.0104815,
        0.00895726, -0.00357939, 0.999955, 0.120179,
        0, 0, 0, 1;
    // 绕z旋转180
    // T1 << -1, 0, 0, 0,
    //     0, -1, 0, 0,
    //     0, 0, 1, 0,
    //     0, 0, 0, 1;
    // Eigen::Matrix4f T_init;
    // T_init=T1*T;

    // pcl::VoxelGrid<pcl::PointXYZ> vg;
    // vg.setInputCloud(cloud_source);
    // vg.setLeafSize(0.0003, 0.0003, 0.0003);
    // vg.filter(*cloud_source);
    // std::cout << "目标点云滤波后还剩 " << cloud_source->size() << " 个点" << endl;

    // pcl::StatisticalOutlierRemoval<PointT> sor;
    // sor.setInputCloud(cloud_source);
    // sor.setMeanK(40);
    // sor.setStddevMulThresh(1);
    // sor.setNegative(false);
    // sor.filter(*cloud_source);
    // std::cout << cloud_target->size() << std::endl;
    // std::cout << cloud_source->size() << std::endl;
    // Eigen::Matrix3d R_eye_in_hand;
    // Eigen::Vector3d tcp(-28.26 / 1000, -18.89 / 1000, 271.00 / 1000);
    // Eigen::Vector3d t_eye_in_hand(64.723506486 / 1000, -0.58450654 / 1000, 32.056097741 / 1000);
    // R_eye_in_hand << 0.0052907999, -0.999982504, -0.00264552,
    //     0.999885499, 0.005327757, -0.01416346719,
    //     0.014177314, -0.0025702819, 0.99989619;
    // // std::cout<<"R_eye_in_hand::"<<R_eye_in_hand*R_eye_in_hand.inverse()<<std::endl;

    // // 保存roi点云到tcp坐标系下
    // Eigen::Matrix4d T_to_tcp;
    // T_to_tcp.setIdentity();
    // Eigen::Isometry3d T_to_tcp_m3 = Eigen::Isometry3d::Identity();
    // Eigen::Vector3d t_to_tcp = t_eye_in_hand - tcp;
    // T_to_tcp_m3.rotate(R_eye_in_hand);
    // T_to_tcp_m3.pretranslate(t_to_tcp);
    // // 方法2
    // /*
    // T_to_tcp.block<3,3>(0,0)=R_eye_in_hand;
    // T_to_tcp.topRightCorner<3,1>()=t_to_tcp;
    // std::cout<<"T_to_tcpIsometry3d: "<<T_to_tcp_m3.matrix()<<"------------"<<std::endl; */
    // T_to_tcp = T_to_tcp_m3.matrix();
    // // pcl::transformPointCloud(*cloud_source, *cloud_source, T_to_tcp.cast<float>());

    pcl::IterativeClosestPoint<PointT, PointT> icp, icp_secondary;

    icp.setInputSource(cloud_source);
    icp.setInputTarget(cloud_target);
    icp.setTransformationEpsilon(1e-10);    // 为终止条件设置最小转换差异
    icp.setMaxCorrespondenceDistance(0.2); // 设置对应点对之间的最大距离（大于该距离的点不考虑 m。
    icp.setEuclideanFitnessEpsilon(0.001);  // 设置收敛条件是均方误差和小于阈值， 停止迭代；
    icp.setMaximumIterations(35);           // 最大迭代次数

    PointCloudT::Ptr cloud_icp(new PointCloudT());
    PointCloudT::Ptr cloud_icp_2(new PointCloudT());

    std::cout << "start align with trans_init" << std::endl;
    icp.align(*cloud_icp,T);
    std::cout << "icp.getFinalTransformation())------------ :\n"
              << icp.getFinalTransformation() << std::endl;

    icp_secondary.setInputSource(cloud_source);
    icp_secondary.setInputTarget(cloud_target);
    icp_secondary.setTransformationEpsilon(1e-10);      // 为终止条件设置最小转换差异
    icp_secondary.setMaxCorrespondenceDistance(0.001);  // 设置对应点对之间的最大距离（大于该距离的点不考虑 m。
    icp_secondary.setEuclideanFitnessEpsilon(0.0001); // 设置收敛条件是均方误差和小于阈值， 停止迭代；
    icp_secondary.setMaximumIterations(50);
    icp_secondary.align(*cloud_icp_2, icp.getFinalTransformation());
    std::cout << "icp_secondary.getFinalTransformation())------------ :\n"
              << icp_secondary.getFinalTransformation() << std::endl;
    // pcl::io::savePCDFile("/home/ahpc/myspace/pcl_tool/pcl_tool/GB_charger_zeek.pcd", *cloud_icp_2);
    /*icp_secondary.getFinalTransformation())------------ :
    0.99856   0.0142509   0.0517843 -0.00442908
 -0.0103793    0.997184  -0.0742796  -0.0172684
  -0.052697    0.073635    0.995892  0.00394122
          0           0           0           1*/
    bool show_icp = true;
    if (show_icp)
    {
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer1(new pcl::visualization::PCLVisualizer("/icp Viewer,red==cloud_source ,green==template_cloud,blue==icp_cloud"));
        viewer1->setBackgroundColor(0.5, 0.5, 0.5);
        // // // viewer->addPointCloud<PointT>(cloud, "sample cloud");
        pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_source_color_h(cloud_source, 255, 0, 0);
        viewer1->addPointCloud(cloud_source, cloud_source_color_h, "cloud_target"); //  设置cloud_target点云为红色
        pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_tr_color_h(cloud_target, 0, 255, 0);
        viewer1->addPointCloud(cloud_target, cloud_tr_color_h, "cloud_source"); //  设置cloud_source点云为绿色
        pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_icp_color_h(cloud_icp_2, 0, 0, 255);
        // pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_icp_2_color_h(cloud_icp_2, 0, 255, 255);

        viewer1->addPointCloud(cloud_icp_2, cloud_icp_color_h, "cloud_icp"); ////  设置cloud_icp点云为蓝色
        // viewer1->addPointCloud(cloud_icp_2, cloud_icp_2_color_h, "cloud_icp_2"); ////  设置cloud_icp点云为蓝色

        viewer1->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud_target");
        viewer1->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud_source");
        viewer1->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud_icp");

        // viewer1->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 3, "cloud_icp");

        viewer1->addCoordinateSystem(0.1);

        while (!viewer1->wasStopped())
        {
            viewer1->spinOnce(100);
            // icp.align(*cloud_source);
            // boost::this_thread::sleep(boost::posix_time::microseconds(100000));
        }
    }
}
