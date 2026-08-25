cmake_minimum_required(VERSION 3.8)
project(tools)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(amz_vio_pipeline_msgs REQUIRED)

add_executable(calibration_timestamps_to_csv src/calibration_timestamps_to_csv.cpp)

ament_target_dependencies(calibration_timestamps_to_csv
  rclcpp
  rosbag2_cpp
  rosbag2_storage
  amz_vio_pipeline_msgs
)

install(TARGETS
  calibration_timestamps_to_csv
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
