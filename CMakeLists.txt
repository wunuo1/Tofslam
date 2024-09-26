cmake_minimum_required(VERSION 3.14)
project(ct_lio)

MESSAGE(STATUS "[Release Mode]")
set(CMAKE_BUILD_TYPE "Release")

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


#set(DNN_PATH "/usr/")
#set(APPSDK_PATH "/usr/")

#set(DNN_LIB_PATH ${DNN_PATH}/lib)
#set(APPSDK_LIB_PATH ${APPSDK_PATH}/lib/hbbpu)
#set(BPU_libs dnn cnn_intf hbrt_bernoulli_aarch64)

#include_directories(${DNN_PATH}/include
#                    ${APPSDK_PATH}/include)
#link_directories(${DNN_LIB_PATH}
#                ${APPSDK_PATH}/lib/hbbpu
#                ${APPSDK_PATH}/lib)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake)

find_package(G2O REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosbag2_cpp  REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(Boost REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(SYSTEM ${G2O_INCLUDE_DIR} ${G2O_INCLUDE_DIRS} /usr/include/opencv4)

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

ament_export_dependencies(${OpenCV_LIBS} ${Boost_LIBRARIES} glog gflags Eigen3::Eigen3 rclcpp nav_msgs cv_bridge g2o pcl_conversions tf2 tf2_ros image_transport)

include_directories(
  /opt/ros/humble/include
  /usr/include/eigen3
  /usr/include/pcl-1.12
  /usr/include/vtk-9.1
)

include_directories(${PROJECT_SOURCE_DIR})
include_directories(${PROJECT_SOURCE_DIR}/src)
include_directories(${PROJECT_SOURCE_DIR}/thirdparty/sophus)
include_directories(${PROJECT_SOURCE_DIR}/thirdparty/tessil-src/include)
include_directories(${PROJECT_SOURCE_DIR}/thirdparty ${Boost_INCLUDE_DIRS})

include_directories(
  include
)
link_directories(${G2O_LIBRARIES})
link_directories(${OpenCV_LIBRARY_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|i386|x86")
  include_directories(${PROJECT_SOURCE_DIR}/thirdparty/onnxruntime-linux-x64-1.9.0/include)
  link_directories(${PROJECT_SOURCE_DIR}/thirdparty/onnxruntime-linux-x64-1.9.0/lib)
  link_directories(/usr/lib/x86_64-linux-gnu)
  add_definitions(-DPLATFORM_X86)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "arm|aarch64")
  include_directories(${PROJECT_SOURCE_DIR}/thirdparty/onnxruntime-linux-aarch64-1.16.0/include)
  link_directories(${PROJECT_SOURCE_DIR}/thirdparty/onnxruntime-linux-aarch64-1.16.0/lib)
else()
  message(STATUS "Unknown architecture: ${CMAKE_SYSTEM_PROCESSOR}")
endif()

find_library(BOOST_SERIALIZATION boost_serialization)
if (NOT BOOST_SERIALIZATION)
    message(FATAL_ERROR "Can't find libboost_serialization")
endif()
link_directories(${BOOST_SERIALIZATION})

add_subdirectory(src)

