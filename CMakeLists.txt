cmake_minimum_required(VERSION 3.5)
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

#find_package(G2O REQUIRED)
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
#find_package(eigen_conversions REQUIRED)
#find_package(message_generation REQUIRED)
find_package(Boost REQUIRED)


add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

add_executable(main_eskf src/apps/main_eskf.cpp)

target_link_libraries(main_eskf ${OpenCV_LIBS} ${Boost_LIBRARIES} Eigen3::Eigen)

ament_target_dependencies(main_eskf
  rclcpp
  rclpy
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  rosbag2_cpp 
  cv_bridge
  image_transport
  #eigen_conversions
  #message_generation
)



include_directories(
  include
  /opt/ros/foxy/include
  /usr/include/eigen3
  /usr/include/pcl-1.10
  /usr/include/vtk-7.1
)
include_directories(${PROJECT_SOURCE_DIR})
include_directories(${PROJECT_SOURCE_DIR}/src)
#include_directories(${PROJECT_SOURCE_DIR}/thirdparty/sophus)
include_directories(${PROJECT_SOURCE_DIR}/thirdparty/onnxruntime-linux-x64-1.9.0/include)
include_directories(${PROJECT_SOURCE_DIR}/thirdparty/tessil-src/include)
include_directories(${PROJECT_SOURCE_DIR}/thirdparty ${Boost_INCLUDE_DIRS})

include_directories(SYSTEM ${G2O_INCLUDE_DIR} ${G2O_INCLUDE_DIRS} /usr/include/opencv4)

link_directories(${G2O_LIBRARY_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
link_directories(${catkin_LIBRARY_DIRS})
link_directories(${PROJECT_SOURCE_DIR}/thirdparty/onnxruntime-linux-x64-1.9.0/lib)

find_library(BOOST_SERIALIZATION boost_serialization)
if (NOT BOOST_SERIALIZATION)
    message(FATAL_ERROR "Can't find libboost_serialization")
endif()
link_directories(${BOOST_SERIALIZATION})

add_subdirectory(src)



