cmake_minimum_required(VERSION 2.8)

project(pcl_tools)

find_package(PCL 1.7)
include_directories("/usr/include/eigen3")
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# add_executable(eigen_test eigen_test.cpp)
# target_link_libraries(eigen_test ${PCL_LIBRARIES})

add_executable(area_pick area_pick.cpp)
target_link_libraries(area_pick ${PCL_LIBRARIES})

add_executable(normal_estimation normal_estimation.cpp)
target_link_libraries(normal_estimation ${PCL_LIBRARIES})

add_executable(ply_to_pcd ply_to_pcd.cpp)
target_link_libraries(ply_to_pcd ${PCL_LIBRARIES})

add_executable(template_alignment template_alignment.cpp)
target_link_libraries(template_alignment ${PCL_LIBRARIES})

add_executable(resampling resampling.cpp)
target_link_libraries(resampling ${PCL_LIBRARIES})


add_executable(pcd_to_txt pcd_to_txt.cpp)
target_link_libraries(pcd_to_txt ${PCL_LIBRARIES})

add_executable(trajectory_cloud_extract trajectory_cloud_extract.cpp)
target_link_libraries(trajectory_cloud_extract ${PCL_LIBRARIES})

