cmake_minimum_required(VERSION 3.8)
project(can_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(amz_vio_pipeline_msgs REQUIRED)
find_package(orchestrator REQUIRED)

# executable
add_executable(can_driver src/can_driver.cpp)

# add dependencies
ament_target_dependencies(${PROJECT_NAME}
                          rclcpp
                          std_msgs
                          sensor_msgs
                          amz_vio_pipeline_msgs
                          orchestrator)

# expose header file to cpp file
target_include_directories(can_driver 
                           PUBLIC
                           $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                           $<INSTALL_INTERFACE:include>)

# link libraries 
target_link_libraries(${PROJECT_NAME}
                      orchestrator::orchestrator_utils
)

# Expose executable for ROS2 to find
install(TARGETS
        can_driver
        DESTINATION lib/${PROJECT_NAME})

ament_package()
