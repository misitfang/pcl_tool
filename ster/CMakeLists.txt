cmake_minimum_required(VERSION 3.11)
project(yolov8_detect_demo)
set (CMAKE_CXX_STANDARD 14)

set (OpenCV_DIR /usr/local/opencv/opencv-4.5.5/lib/cmake/opencv4)
find_package(OpenCV 4 REQUIRED)
message(STATUS "OpenCV library status:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBS})
set(OPENCV_NEED_LIBS opencv_core opencv_highgui opencv_imgcodecs opencv_imgproc opencv_video opencv_videoio)

find_package(Threads REQUIRED)

# PCL
find_package(PCL REQUIRED)
message(STATUS "PCL library status:")
message(STATUS "    version: ${PCL_VERSION}")
include_directories( ${PCL_INCLUDE_DIRS} )
find_package (glog REQUIRED)

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

include_directories(${CMAKE_SOURCE_DIR}/include/ahpc_vision)

include_directories("/usr/include/eigen3")

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/mvg-pc-generator)
add_library(get_final_result test/get_final_result.cpp)


set(COMMON_SOURCES common/MatViewer.cpp common/TYThread.cpp common/ImageSpeckleFilter.cpp common/DepthInpainter.cpp)
add_library(sample_common STATIC ${COMMON_SOURCES})
if(UNIX)
  target_link_libraries(sample_common pthread)
endif()

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/jaka)
link_directories(${CMAKE_SOURCE_DIR}/libs/jaka)

if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "x86_64")
    message(STATUS "USE x86_64 PROCESSOR")
    include_directories(${CMAKE_SOURCE_DIR}/include/ncnn)
    link_directories(${CMAKE_SOURCE_DIR}/libs/ncnn)
    link_directories(${CMAKE_SOURCE_DIR}/libs/ahpc_vision/x86)
    add_executable(${PROJECT_NAME}_charger_port ${CMAKE_SOURCE_DIR}/test/main_charger_port.cpp)
    target_link_libraries(${PROJECT_NAME}_charger_port ${OPENCV_NEED_LIBS} yolov8n_ncnn_sdk ncnn)# link main sdk
    add_executable(${PROJECT_NAME}_charger ${CMAKE_SOURCE_DIR}/test/main_charger.cpp)
    target_link_libraries(${PROJECT_NAME}_charger ${OPENCV_NEED_LIBS} yolov8n_ncnn_sdk ncnn)# link main sdk  
    add_executable(${PROJECT_NAME}_car_status ${CMAKE_SOURCE_DIR}/test/main_car_status.cpp)
    target_link_libraries(${PROJECT_NAME}_car_status ${OPENCV_NEED_LIBS} yolov8n_ncnn_sdk ncnn)# link main sdk  
else()
    message(STATUS "USE ARM PROCESSOR")
    #for aarch64 rknn
    include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/ty)
    link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib/linux/lib_Aarch64)
    link_directories(${CMAKE_SOURCE_DIR}/libs/ahpc_vision/aarch64)
    link_directories(${CMAKE_SOURCE_DIR}/libs/mvg-pc-generator)
    include_directories(${CMAKE_CURRENT_SOURCE_DIR}/robot/jaka/include/jaka_api)
    link_directories(${CMAKE_SOURCE_DIR}/robot/jaka/pylib/aarch64)
    include_directories(${CMAKE_CURRENT_SOURCE_DIR}/3rdlib/include)
    include_directories(${CMAKE_CURRENT_SOURCE_DIR}/3rdlib/include/RansacLib)
    link_directories(${CMAKE_CURRENT_SOURCE_DIR}/3rdlib/lib)
    # add_executable(${PROJECT_NAME}_charger_port ${CMAKE_SOURCE_DIR}/test/main_charger_port.cpp)
    # target_link_libraries(${PROJECT_NAME}_charger_port ${OPENCV_NEED_LIBS} yolov8n_rknn_sdk)# link main sdk
    add_executable(${PROJECT_NAME}_charger ${CMAKE_SOURCE_DIR}/test/main_charger.cpp)
    target_link_libraries(${PROJECT_NAME}_charger ${CMAKE_CURRENT_SOURCE_DIR}/lib/linux/lib_Aarch64/libtycam.so)
    target_link_libraries(${PROJECT_NAME}_charger ${OPENCV_NEED_LIBS} robust_pose_pnp  SQPnP yolov8n_rknn_sdk get_final_result mvg_pc_generator g2o ${PCL_LIBRARIES} sample_common jakaAPI soft_inserter_interface glog)# link main sdk 
    # add_executable(${PROJECT_NAME}_car_status ${CMAKE_SOURCE_DIR}/test/main_car_status.cpp)
    # target_link_libraries(${PROJECT_NAME}_car_status ${OPENCV_NEED_LIBS} yolov8n_rknn_sdk)# link main sdk 
endif()
