cmake_minimum_required(VERSION 2.8)

project(normal_est)

set(CMAKE_CXX_FLAGS_DEBUG "{CMAKE_CXX_FLAGS} -o0 -ggbd")    #添加语句
add_compile_options(-g)                                     #添加语句
SET(CMAKE_BUILD_TYPE “DEBUG”)

find_package(PCL 1.7)
include_directories("/usr/include/eigen3")
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set (OpenCV_DIR /usr/local/opencv)
find_package(OpenCV 4 REQUIRED)
message(STATUS "OpenCV library status:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/3rd/include/ncnn)
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/3rd/lib)
link_directories(/home/ahpc/workspace/yolov5s_ncnn_0117/yolov5s_ncnn/3rd/lib/)
# find_package(OpenCV 4 REQUIRED)
# include_directories(
#   include
#   ${OpenCV_INCLUDE_DIR}
# )
# link_directories(${OpenCV_LIBS})

# add_executable(eigen_test eigen_test.cpp)
# target_link_libraries(eigen_test ${PCL_LIBRARIES})

add_executable(area_pick src/area_pick.cpp)
target_link_libraries(area_pick ${PCL_LIBRARIES})

add_executable(normal_estimation src/normal_estimation.cpp)
target_link_libraries(normal_estimation ${PCL_LIBRARIES})

# include_directories(so_test.h)
# add_library(so_test SHARED so_test.cpp)
# add_executable(so_test_main so_test_main.cpp)
# target_link_libraries(so_test_main ${OpenCV_LIBS})
# target_link_libraries(so_test_main so_test yolov5_ncnn ncnn)

# target_link_libraries(so_test_main "/home/ahpc/workspace/yolov5s_ncnn_0117/yolov5s_ncnn/3rd/lib/libncnn.so")

add_executable(xyz_to_pcd src/xyz_to_pcd.cpp)
target_link_libraries(xyz_to_pcd ${PCL_LIBRARIES})

add_executable(ply_to_pcd src/ply_to_pcd.cpp)
target_link_libraries(ply_to_pcd ${PCL_LIBRARIES})


add_executable(ransac_registration src/ransac_registration.cpp)
target_link_libraries(ransac_registration ${PCL_LIBRARIES})

add_executable(icp_test src/icp_test.cpp)
target_link_libraries(icp_test ${PCL_LIBRARIES})

add_executable(icp_p_plane src/icp_p_plane.cpp)
target_link_libraries(icp_p_plane ${PCL_LIBRARIES})

add_executable(corss src/corss.cpp)
target_link_libraries(corss ${PCL_LIBRARIES})

add_executable(alignment_prerejective src/alignment_prerejective.cpp)
target_link_libraries(alignment_prerejective ${PCL_LIBRARIES})


add_executable(region_growing_segmentation src/region_growing_segmentation.cpp)
target_link_libraries(region_growing_segmentation ${PCL_LIBRARIES})

add_executable(euclidean_seg src/euclidean_seg.cpp)
target_link_libraries(euclidean_seg ${PCL_LIBRARIES})

add_executable(model_trans src/model_trans.cpp)
target_link_libraries(model_trans ${PCL_LIBRARIES})

add_executable(icp_process src/icp_process.cpp)
target_link_libraries(icp_process ${PCL_LIBRARIES})


find_package (glog REQUIRED)
add_executable(glog_test src/glog_test.cpp)
target_link_libraries(glog_test ${PCL_LIBRARIES} glog)


add_executable(icp src/icp.cpp)
target_link_libraries(icp ${PCL_LIBRARIES} glog)
