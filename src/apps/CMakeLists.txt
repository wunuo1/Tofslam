add_executable(${PROJECT_NAME}_eskf main_eskf.cpp  
                               common_utility.cpp)

find_package(Threads REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(fmt REQUIRED)
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
			fmt::fmt
                        )

if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|i386|x86")
  target_link_libraries(${PROJECT_NAME}_eskf ${PROJECT_SOURCE_DIR}/thirdparty/onnxruntime-linux-x64-1.9.0/lib/libonnxruntime.so.1.9.0)
  message(STATUS "Current system is x86 architecture.")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "arm|aarch64")
  target_link_libraries(${PROJECT_NAME}_eskf ${PROJECT_SOURCE_DIR}/thirdparty/onnxruntime-linux-aarch64-1.16.0/lib/libonnxruntime.so.1.16.0)
  message(STATUS "Current system is arm architecture.")
else()
  message(STATUS "Unknown architecture: ${CMAKE_SYSTEM_PROCESSOR}")
endif()



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

if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|i386|x86")
  install(FILES ${PROJECT_SOURCE_DIR}/thirdparty/onnxruntime-linux-x64-1.9.0/lib/libonnxruntime.so.1.9.0
  DESTINATION lib)
  message(STATUS "Current system is x86 architecture.")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "arm|aarch64")
  install(FILES ${PROJECT_SOURCE_DIR}/thirdparty/onnxruntime-linux-aarch64-1.16.0/lib/libonnxruntime.so.1.16.0
  DESTINATION lib)
  message(STATUS "Current system is arm architecture.")
else()
  message(STATUS "Unknown architecture: ${CMAKE_SYSTEM_PROCESSOR}")
endif()

install(DIRECTORY
${PROJECT_SOURCE_DIR}/launch/
DESTINATION share/${PROJECT_NAME}/launch)

ament_package()
