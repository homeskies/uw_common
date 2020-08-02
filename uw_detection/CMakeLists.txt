cmake_minimum_required(VERSION 2.8.12)
project(uw_detection)

find_package(catkin REQUIRED COMPONENTS   cv_bridge
        geometry_msgs
        image_transport
        message_generation
        message_filters
        message_runtime
        roscpp
        roslint
        sensor_msgs
        std_msgs
        std_srvs
        tf2
        tf2_eigen
        visualization_msgs)

catkin_python_setup()

catkin_package()


option(CUDA_USE_STATIC_CUDA_RUNTIME OFF)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Debug)

find_package(OpenCV)
include_directories(OpenCV_INCLUDE_DIRS
        ${catkin_INCLUDE_DIRS}
        include)

find_package(CUDA)

if (CUDA_FOUND)
set(CUDA_NVCC_PLAGS ${CUDA_NVCC_PLAGS};-std=c++11;-g;-G;-gencode;arch=compute_30;code=sm_30)


if (CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    message("embed_platform on")
    include_directories(/usr/local/cuda/targets/aarch64-linux/include)
    link_directories(/usr/local/cuda/targets/aarch64-linux/lib)
else()
    message("embed_platform off")
    include_directories(/usr/local/cuda/include)
    link_directories(/usr/local/cuda/lib64)
endif()


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Ofast -Wfatal-errors -D_MWAITXINTRIN_H_INCLUDED")

cuda_add_library(yololayer SHARED src/yolov3-spp/yololayer.cu)

add_library(yolov3-spp src/yolov3-spp/yolov3-spp.cpp)
target_link_libraries(yolov3-spp nvinfer cudart yololayer ${OpenCV_LIBS})

add_library(yolov3 src/yolov3-spp/yolov3.cpp)
target_link_libraries(yolov3 nvinfer cudart yololayer ${OpenCV_LIBS})

add_executable(yolov3_spp_prepare_plan src/yolov3_spp_prepare_plan.cpp)
target_link_libraries(yolov3_spp_prepare_plan yolov3-spp nvinfer cudart yololayer ${OpenCV_LIBS} ${catkin_LIBRARIES})

add_executable(yolov3_prepare_plan src/yolov3_prepare_plan.cpp)
target_link_libraries(yolov3_prepare_plan yolov3 nvinfer cudart yololayer ${OpenCV_LIBS} ${catkin_LIBRARIES})

add_executable(yolo_ros_image_node src/yolo_ros_image_node.cpp)
target_link_libraries(yolo_ros_image_node yolov3 nvinfer cudart yololayer ${OpenCV_LIBS} ${catkin_LIBRARIES})

add_definitions(-O2 -pthread)
endif()

