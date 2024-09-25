
add_library(${PROJECT_NAME}.common
        timer/timer.cc
        utility.cpp
        utils_trans.cpp
        utils.cpp
        global_flags.cc
        read_onnx.cpp
        )
ament_target_dependencies(
        ${PROJECT_NAME}.common
        pcl_conversions)
target_link_libraries(${PROJECT_NAME}.common
        ${third_party_libs}
        )