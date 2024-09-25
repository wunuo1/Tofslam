add_executable(${PROJECT_NAME}_eskf main_eskf.cpp  
                               common_utility.cpp)

find_package(Threads REQUIRED)
find_package(ament_cmake REQUIRED)

target_link_libraries(${PROJECT_NAME}_eskf 
                         ${PROJECT_NAME}.common
                         ${PROJECT_NAME}.pre
                         ${PROJECT_NAME}.liw
                         ${PROJECT_NAME}.algo
                         ${PROJECT_NAME}.loop
                         ${third_party_libs}
                         ${CMAKE_THREAD_LIBS_INIT}
                        ${OpenCV_LIBS}
                        yaml-cpp
                        ceres
                        ${PROJECT_SOURCE_DIR}/thirdparty/onnxruntime-linux-x64-1.9.0/lib/libonnxruntime.so.1.9.0
                        )
message(${PROJECT_SOURCE_DIR})


link_directories(${G2O_LIBRARY_DIRS})

ament_target_dependencies(
    ${PROJECT_NAME}_eskf
    rclcpp
    cv_bridge
    nav_msgs
    pcl_conversions
    tf2_ros
    image_transport
)

install(TARGETS ${PROJECT_NAME}_eskf
  DESTINATION lib/${PROJECT_NAME})

install(FILES ${PROJECT_SOURCE_DIR}/thirdparty/onnxruntime-linux-x64-1.9.0/lib/libonnxruntime.so.1.9.0
DESTINATION /opt/ros/humble/lib)

install(DIRECTORY
${PROJECT_SOURCE_DIR}/launch/
DESTINATION share/${PROJECT_NAME}/launch)

ament_package()
