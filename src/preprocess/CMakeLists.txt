
add_library(${PROJECT_NAME}.pre
        cloud_convert/cloud_convert.cc
        )
ament_target_dependencies(
        ${PROJECT_NAME}.pre
        pcl_conversions)
target_link_libraries(${PROJECT_NAME}.pre
        ${third_party_libs}
        )
