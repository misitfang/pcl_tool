#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

#include <pcl/common/time.h>//计算时间
#include <pcl/console/print.h>//PCL控制台输出

#include <pcl/features/normal_3d_omp.h>//使用OMP需要添加的头文件
#include <pcl/features/fpfh.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/visualization/pcl_visualizer.h>

// Types
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointCloudT;
typedef pcl::FPFHSignature33 FeatureT;
typedef pcl::FPFHEstimation<PointNT,PointNT,FeatureT> FeatureEstimationT;
typedef pcl::PointCloud<FeatureT> FeatureCloudT;
typedef pcl::visualization::PointCloudColorHandlerCustom<PointNT> ColorHandlerT;

// 将一个刚性物体与一个带有杂波和遮挡的场景对齐
int
main (int argc, char **argv)
{
  // Point clouds
  PointCloudT::Ptr object (new PointCloudT);
  PointCloudT::Ptr object_aligned (new PointCloudT);
  PointCloudT::Ptr scene (new PointCloudT);
  FeatureCloudT::Ptr object_features (new FeatureCloudT);
  FeatureCloudT::Ptr scene_features (new FeatureCloudT);

  
  //  加载目标物体和场景点云
  pcl::console::print_highlight ("加载点云数据\n");

  if (pcl::io::loadPCDFile<PointNT> ("/home/ahpc/myspace/pcl_tool/pcl_tool/models/mechmuban2.pcd", *object) < 0 ||
      pcl::io::loadPCDFile<PointNT> ("/home/ahpc/myspace/pcl_tool/pcl_tool/models/pcl_G9_model.pcd", *scene) < 0)
  {
    pcl::console::print_error ("点云加载失败!\n");
    return (1);
  }
       
  // 下采样
  pcl::console::print_highlight ("点云下采样\n");
  pcl::VoxelGrid<PointNT> grid;
  const float leaf = 0.005f;
  grid.setLeafSize (leaf, leaf, leaf);
  grid.setInputCloud (object);
  grid.filter (*object);
  grid.setInputCloud (scene);
  grid.filter (*scene);
  
  // 估计场景法线
  pcl::console::print_highlight ("估计场景点云的法线\n");
  pcl::NormalEstimationOMP<PointNT,PointNT> n;
  n.setNumberOfThreads(4);//设置openMP的线程数
  n.setRadiusSearch (0.01);
  n.setInputCloud (scene);
  n.compute (*scene);
  
  // 特征估计
  pcl::console::print_highlight ("计算FPFH特征\n");
  FeatureEstimationT f;
  f.setRadiusSearch (0.025);
  f.setInputCloud (object);
  f.setInputNormals (object);
  f.compute (*object_features);
  f.setInputCloud (scene);
  f.setInputNormals (scene);
  f.compute (*scene_features);
  
  // 实施配准
  pcl::console::print_highlight ("开始进行配准\n");
  pcl::SampleConsensusPrerejective<PointNT,PointNT,FeatureT> align;
  align.setInputSource (object);
  align.setSourceFeatures (object_features);
  align.setInputTarget (scene);
  align.setTargetFeatures (scene_features);
  align.setMaximumIterations (50000);      //  采样一致性迭代次数
  align.setNumberOfSamples (3);            //  创建假设所需的样本数
  align.setCorrespondenceRandomness (5);   //  使用的临近特征点的数目
  align.setSimilarityThreshold (0.8f);     //  多边形边长度相似度阈值
  align.setMaxCorrespondenceDistance (2.5f * 0.005); //  判断是否为内点的距离阈值
  align.setInlierFraction (0.25f);         //  接受位姿假设所需的内点比例
  {
    pcl::ScopeTime t("执行配准");
    align.align (*object_aligned);
  }
   pcl::io::savePCDFileASCII ("object_aligned.pcd", *object_aligned);

  if (align.hasConverged ())
  {
    // Print results
    printf ("\n");
    Eigen::Matrix4f transformation = align.getFinalTransformation ();
    pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (0,0), transformation (0,1), transformation (0,2));
    pcl::console::print_info ("R = | %6.3f %6.3f %6.3f | \n", transformation (1,0), transformation (1,1), transformation (1,2));
    pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (2,0), transformation (2,1), transformation (2,2));
    pcl::console::print_info ("\n");
    pcl::console::print_info ("t = < %0.3f, %0.3f, %0.3f >\n", transformation (0,3), transformation (1,3), transformation (2,3));
    pcl::console::print_info ("\n");
    pcl::console::print_info ("Inliers: %i/%i\n", align.getInliers ().size (), object->size ());
    
    // Show alignment
    pcl::console::print_highlight("左侧为配准前的位置，右侧为配准后\n");
    pcl::visualization::PCLVisualizer visu("鲁棒位姿估计");
	int v1(0),v2(0);
    visu.setWindowName("鲁棒位姿估计");
	visu.createViewPort(0,0,0.5,1,v1);
	visu.createViewPort(0.5,0,1,1,v2);
	visu.setBackgroundColor(255,255,255,v2);
    visu.addPointCloud (scene, ColorHandlerT (scene, 0.0, 255.0, 0.0), "scene",v2);
    visu.addPointCloud (object_aligned, ColorHandlerT (object_aligned, 0.0, 0.0, 255.0), "object_aligned",v2);
	
	visu.addPointCloud(object,ColorHandlerT (object, 0.0, 255.0, 0.0), "object_before_aligned",v1);
	visu.addPointCloud(scene,ColorHandlerT (scene, 0.0, 0.0, 255.0), "scene_v2",v1);
	visu.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,3,"scene");
	visu.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,3,"object_aligned");
	visu.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,3,"object_before_aligned");
	visu.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,3,"scene_v2");
    visu.spin ();
  }
  else
  {
    pcl::console::print_error ("配准失败!\n");
    return (1);
  }
  
  return (0);
}
