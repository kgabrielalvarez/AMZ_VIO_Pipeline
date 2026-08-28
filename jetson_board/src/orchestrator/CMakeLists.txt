cmake_minimum_required(VERSION 3.8)
project(orchestrator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(amz_vio_pipeline_msgs REQUIRED)

# executable
add_executable(orchestrator src/orchestrator.cpp)

# create utils library for other nodes to use
add_library(orchestrator_utils
            src/orchestrator_utils.cpp
)

# add dependencies
ament_target_dependencies(${PROJECT_NAME}
                          rclcpp
                          std_msgs
                          amz_vio_pipeline_msgs)

# expose header file to cpp file
target_include_directories(orchestrator_utils
                           PUBLIC
                           $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                           $<INSTALL_INTERFACE:include>)

# link library
target_link_libraries(orchestrator
    orchestrator_utils
)

# Expose executable for ROS2 to find
install(TARGETS orchestrator
        DESTINATION lib/${PROJECT_NAME})

install(
    TARGETS orchestrator_utils
    EXPORT orchestrator_utilsTargets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

ament_export_targets(orchestrator_utilsTargets HAS_LIBRARY_TARGET
                     NAMESPACE orchestrator::)

install(
    DIRECTORY include/
    DESTINATION include
)

install(
    DIRECTORY config
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
