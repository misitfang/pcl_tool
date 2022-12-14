#include <Eigen/Core>
#include <fstream>
#include <limits>
#include <pcl/features/fpfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include <vector>

#include <chrono>

typedef pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> PCLHandler;

class FeatureCloud
{
public:
    // A bit of shorthand
    typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
    typedef pcl::PointCloud<pcl::Normal> SurfaceNormals;
    typedef pcl::PointCloud<pcl::FPFHSignature33> LocalFeatures;
    typedef pcl::search::KdTree<pcl::PointXYZ> SearchMethod;

    FeatureCloud() : search_method_xyz_(new SearchMethod),
                     normal_radius_(0.02f),
                     feature_radius_(0.02f) {}

    ~FeatureCloud() {}

    // Process the given cloud
    void
    setInputCloud(PointCloud::Ptr xyz)
    {
        xyz_ = xyz;
        processInput();
    }

    // Load and process the cloud in the given PCD file
    void
    loadInputCloud(const std::string &pcd_file)
    {
        xyz_ = PointCloud::Ptr(new PointCloud);
        pcl::io::loadPCDFile(pcd_file, *xyz_);
        processInput();
    }

    // Get a pointer to the cloud 3D points
    PointCloud::Ptr
    getPointCloud() const
    {
        return (xyz_);
    }

    // Get a pointer to the cloud of 3D surface normals
    SurfaceNormals::Ptr
    getSurfaceNormals() const
    {
        return (normals_);
    }

    // Get a pointer to the cloud of feature descriptors
    LocalFeatures::Ptr
    getLocalFeatures() const
    {
        return (features_);
    }

protected:
    // Compute the surface normals and local features
    void
    processInput()
    {
        computeSurfaceNormals();
        computeLocalFeatures();
    }

    // Compute the surface normals
    void
    computeSurfaceNormals()
    {
        // ?????????????????????
        normals_ = SurfaceNormals::Ptr(new SurfaceNormals);

        // ?????????????????????
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> norm_est;
        norm_est.setInputCloud(xyz_);
        norm_est.setSearchMethod(search_method_xyz_);
        norm_est.setRadiusSearch(normal_radius_);
        norm_est.compute(*normals_);
    }

    // Compute the local feature descriptors
    /**
     * ????????????????????? ????????????????????????
     */
    void
    computeLocalFeatures()
    {
        features_ = LocalFeatures::Ptr(new LocalFeatures);

        pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_est;
        fpfh_est.setInputCloud(xyz_);
        fpfh_est.setInputNormals(normals_);
        fpfh_est.setSearchMethod(search_method_xyz_);
        fpfh_est.setRadiusSearch(feature_radius_);
        fpfh_est.compute(*features_);
    }

private:
    // Point cloud data
    PointCloud::Ptr xyz_;
    SurfaceNormals::Ptr normals_;
    LocalFeatures::Ptr features_;         // ???????????????????????? Fast Point Feature Histogram
    SearchMethod::Ptr search_method_xyz_; // KDTree??????????????????

    // Parameters
    float normal_radius_;
    float feature_radius_;
};

class TemplateAlignment
{
public:
    // A struct for storing alignment results
    struct Result {
        // ????????????
        float fitness_score;
        // ????????????
        Eigen::Matrix4f final_transformation;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    TemplateAlignment() : min_sample_distance_(0.05f),
                          max_correspondence_distance_(0.01f * 0.01f),
                          nr_iterations_(500)
    {
        // Intialize the parameters in the Sample Consensus Intial Alignment (SAC-IA) algorithm
        sac_ia_.setMinSampleDistance(min_sample_distance_);
        sac_ia_.setMaxCorrespondenceDistance(max_correspondence_distance_);
        sac_ia_.setMaximumIterations(nr_iterations_);
    }

    ~TemplateAlignment() {}

    // Set the given cloud as the target to which the templates will be aligned
    void
    setTargetCloud(FeatureCloud &target_cloud)
    {
        target_ = target_cloud;
        // ????????????target??????
        sac_ia_.setInputTarget(target_cloud.getPointCloud());
        // ????????????target
        sac_ia_.setTargetFeatures(target_cloud.getLocalFeatures());
    }

    // Add the given cloud to the list of template clouds
    void
    addTemplateCloud(FeatureCloud &template_cloud)
    {
        templates_.push_back(template_cloud);
    }

    // Align the given template cloud to the target specified by setTargetCloud ()
    // ?????????????????????
    void
    align(FeatureCloud &template_cloud, TemplateAlignment::Result &result)
    {
        // ???????????????
        sac_ia_.setInputSource(template_cloud.getPointCloud());
        // ???????????????
        sac_ia_.setSourceFeatures(template_cloud.getLocalFeatures());

        pcl::PointCloud<pcl::PointXYZ> registration_output;
        sac_ia_.align(registration_output);

        // ??????????????????????????????????????????
        result.fitness_score = (float)sac_ia_.getFitnessScore(max_correspondence_distance_);
        // ????????????????????????
        result.final_transformation = sac_ia_.getFinalTransformation();
    }

    // Align all of template clouds set by addTemplateCloud to the target specified by setTargetCloud ()
    void
    alignAll(std::vector<TemplateAlignment::Result, Eigen::aligned_allocator<Result>> &results)
    {
        results.resize(templates_.size());
        for (size_t i = 0; i < templates_.size(); ++i) {
            align(templates_[i], results[i]);
        }
    }

    // Align all of template clouds to the target cloud to find the one with best alignment score
    int
    findBestAlignment(TemplateAlignment::Result &result)
    {
        // Align all of the templates to the target cloud
        std::vector<Result, Eigen::aligned_allocator<Result>> results;
        alignAll(results);

        // Find the template with the best (lowest) fitness score
        float lowest_score = std::numeric_limits<float>::infinity();
        int best_template = 0;
        for (size_t i = 0; i < results.size(); ++i) {
            const Result &r = results[i];
            if (r.fitness_score < lowest_score) {
                lowest_score = r.fitness_score;
                best_template = (int)i;
            }
        }

        // Output the best alignment
        result = results[best_template];
        return (best_template);
    }

private:
    // A list of template clouds and the target to which they will be aligned
    std::vector<FeatureCloud> templates_;
    FeatureCloud target_;

    // The Sample Consensus Initial Alignment (SAC-IA) registration routine and its parameters
    pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac_ia_;
    float min_sample_distance_;
    float max_correspondence_distance_;
    int nr_iterations_;
};

/**
 * ?????????????????????????????????????????????
 * ?????????????????? ./template_alignment2 ./data/object_templates.txt ./data/person.pcd
 *                  ??????                          ???????????????????????????         ????????????
 * ?????????????????? ./template_alignment2 ./data/object_templates2.txt ./data/target.pcd
 *                  ??????                          ???????????????????????????         ????????????
 *
 * ?????????????????????RGB????????????
 * ?????????????????????
 * ?????????????????????????????????????????????????????????
 * ???????????????????????????????????????????????????????????????
 */
int main(int argc, char **argv)
{
    if (argc < 3) {
        printf("No target PCD file given!\n");
        return (-1);
    }

    // Load the object templates specified in the object_templates.txt file
    std::vector<FeatureCloud> object_templates;
    std::ifstream input_stream(argv[1]);
    object_templates.resize(0); // vector ??????????????????
    std::string pcd_filename;

    auto startTime = std::chrono::steady_clock::now();

    while (input_stream.good()) //??????????????????????????????
    {
        // ?????????????????????????????????
        std::getline(input_stream, pcd_filename);
        if (pcd_filename.empty() || pcd_filename.at(0) == '#') // Skip blank lines or comments
            continue;

        // ???????????????
        FeatureCloud template_cloud;
        template_cloud.loadInputCloud(pcd_filename);
        object_templates.push_back(template_cloud);
    }
    input_stream.close();

    auto object_templateload_endtime = std::chrono::steady_clock::now();
    auto template_Fileload_time = std::chrono::duration_cast<std::chrono::milliseconds>(object_templateload_endtime - startTime);
    std::cout << "load the object template took " << template_Fileload_time.count() << " milliseconds" << std::endl;

    // Load the target cloud PCD file
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile(argv[2], *cloud);

    auto end_target_fileload_time = std::chrono::steady_clock::now();
    auto target_Fileload_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_target_fileload_time - object_templateload_endtime);
    std::cout << "load the target cloud took " << target_Fileload_time.count() << " milliseconds" << std::endl;
    std::cout << "the size of cloud is " << cloud->size() << std::endl;

    // Preprocess the cloud by...
    // ...removing distant points ?????????????????????

    const float depth_limit = 10.0;
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0, depth_limit);
    pass.filter(*cloud);
    std::cout << "the size of cloud after ps is " << cloud->size() << std::endl;

    // ... and downsampling the point cloud ???????????????, ???????????????
    // ?????????????????? 5mm
    const float voxel_grid_size = 0.001f;
    pcl::VoxelGrid<pcl::PointXYZ> vox_grid;
    vox_grid.setInputCloud(cloud);
    // ???????????????????????????lx, ly, lz
    vox_grid.setLeafSize(voxel_grid_size, voxel_grid_size, voxel_grid_size);
    // vox_grid.filter (*cloud); // Please see this http://www.pcl-developers.org/Possible-problem-in-new-VoxelGrid-implementation-from-PCL-1-5-0-td5490361.html
    pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud(new pcl::PointCloud<pcl::PointXYZ>);
    vox_grid.filter(*tempCloud);
    cloud = tempCloud;
    std::cout << "the size of tempcloud is " << tempCloud->size() << std::endl;
    // ????????????&????????????????????????
    pcl::io::savePCDFileBinary("pass_through_voxel.pcd", *tempCloud);
    std::cout << "pass_through_voxel.pcd saved" << std::endl;

    auto Preprocess_cloud_time = std::chrono::steady_clock::now();
    auto Preprocess_cloud = std::chrono::duration_cast<std::chrono::milliseconds>(Preprocess_cloud_time - end_target_fileload_time);
    std::cout << "preprocess cloud took " << Preprocess_cloud.count() << " milliseconds" << std::endl;

    // Assign to the target FeatureCloud ???????????????????????????
    FeatureCloud target_cloud;
    target_cloud.setInputCloud(cloud);

    // Set the TemplateAlignment inputs
    TemplateAlignment template_align;
    for (size_t i = 0; i < object_templates.size(); i++) {
        FeatureCloud &object_template = object_templates[i];
        // ??????????????????
        template_align.addTemplateCloud(object_template);
    }
    // ??????????????????
    template_align.setTargetCloud(target_cloud);

    std::cout << "findBestAlignment" << std::endl;
    // Find the best template alignment
    // ????????????
    TemplateAlignment::Result best_alignment;
    int best_index = template_align.findBestAlignment(best_alignment);
    const FeatureCloud &best_template = object_templates[best_index];

    auto template_alignment_time = std::chrono::steady_clock::now();
    auto template_alihnment_time_ = std::chrono::duration_cast<std::chrono::milliseconds>(template_alignment_time - Preprocess_cloud_time);
    std::cout << "template_alihnment cloud took " << template_alihnment_time_.count() << " milliseconds" << std::endl;

    // Print the alignment fitness score (values less than 0.00002 are good)
    printf("Best fitness score: %f\n", best_alignment.fitness_score);
    printf("Best fitness best_index: %d\n", best_index);

    // Print the rotation matrix and translation vector
    Eigen::Matrix3f rotation = best_alignment.final_transformation.block<3, 3>(0, 0);
    Eigen::Vector3f translation = best_alignment.final_transformation.block<3, 1>(0, 3);

    Eigen::Vector3f euler_angles = rotation.eulerAngles(2, 1, 0) * 180 / M_PI;

    printf("\n");
    printf("    | %6.3f %6.3f %6.3f | \n", rotation(0, 0), rotation(0, 1), rotation(0, 2));
    printf("R = | %6.3f %6.3f %6.3f | \n", rotation(1, 0), rotation(1, 1), rotation(1, 2));
    printf("    | %6.3f %6.3f %6.3f | \n", rotation(2, 0), rotation(2, 1), rotation(2, 2));
    printf("\n");
    cout << "yaw(z) pitch(y) roll(x) = " << euler_angles.transpose() << endl;
    printf("\n");
    printf("t = < %0.3f, %0.3f, %0.3f >\n", translation(0), translation(1), translation(2));

    // Save the aligned template for visualization
    pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
    // ?????????????????????????????????????????????????????????????????????????????????transformed_cloud
    pcl::transformPointCloud(*best_template.getPointCloud(), transformed_cloud, best_alignment.final_transformation);

    //    pcl::io::savePCDFileBinary("output.pcd", transformed_cloud);
    // =============================================================================

    pcl::visualization::PCLVisualizer viewer("example");
    // ?????????????????????
    viewer.addCoordinateSystem(0.5, "cloud", 0);
    // ???????????????
    viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey

    // 1. ??????????????????rotated --------------------------------
    pcl::PointCloud<pcl::PointXYZ>::Ptr t_cloud(&transformed_cloud);
    PCLHandler transformed_cloud_handler(t_cloud, 255, 0, 0);
    viewer.addPointCloud(t_cloud, transformed_cloud_handler, "transformed_cloud");
    // ?????????????????????????????????
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "transformed_cloud");

    // 2. ????????????target --------------------------------
    PCLHandler target_cloud_handler(cloud, 0, 255, 0);
    viewer.addPointCloud(cloud, target_cloud_handler, "target_cloud");
    // ?????????????????????????????????
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "target_cloud");

    // 3. ????????????template --------------------------------
    PCLHandler template_cloud_handler(cloud, 0, 0, 255);
    viewer.addPointCloud(best_template.getPointCloud(), template_cloud_handler, "template_cloud");
    // ?????????????????????????????????
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "template_cloud");

    auto endtime = std::chrono::steady_clock::now();
    auto cal_time = std::chrono::duration_cast<std::chrono::milliseconds>(endtime - startTime);
    std::cout << "Program run time took" << cal_time.count() << "milliseconds" << std::endl;

    while (!viewer.wasStopped()) { // Display the visualiser until 'q' key is pressed
        viewer.spinOnce();
    }

    return (0);
}
