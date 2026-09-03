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
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(calibration_timestamps_to_csv src/calibration_timestamps_to_csv.cpp)

add_executable(imu_data_to_csv src/imu_data_to_csv.cpp)

add_executable(left_camera_data_to_csv src/left_camera_data_to_csv.cpp)

add_executable(right_camera_data_to_csv src/right_camera_data_to_csv.cpp)

add_executable(left_camera_data_to_png src/left_camera_data_to_png.cpp)

add_executable(right_camera_data_to_png src/right_camera_data_to_png.cpp)

ament_target_dependencies(calibration_timestamps_to_csv
  rclcpp
  rosbag2_cpp
  rosbag2_storage
  amz_vio_pipeline_msgs
)

ament_target_dependencies(imu_data_to_csv
  rclcpp
  rosbag2_cpp
  rosbag2_storage
  sensor_msgs
)

ament_target_dependencies(left_camera_data_to_csv
  rclcpp
  rosbag2_cpp
  rosbag2_storage
  sensor_msgs
)

ament_target_dependencies(right_camera_data_to_csv
  rclcpp
  rosbag2_cpp
  rosbag2_storage
  sensor_msgs
)

ament_target_dependencies(left_camera_data_to_png
  rclcpp
  rosbag2_cpp
  rosbag2_storage
  sensor_msgs
  cv_bridge
)

ament_target_dependencies(right_camera_data_to_png
  rclcpp
  rosbag2_cpp
  rosbag2_storage
  sensor_msgs
  cv_bridge
)

install(TARGETS
  calibration_timestamps_to_csv
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  imu_data_to_csv
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  left_camera_data_to_csv
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  right_camera_data_to_csv
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  left_camera_data_to_png
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  right_camera_data_to_png
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
