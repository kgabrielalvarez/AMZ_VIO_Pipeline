cmake_minimum_required(VERSION 3.8)
project(camera_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pylon REQUIRED)

# create executable
add_executable(${PROJECT_NAME} src/${PROJECT_NAME}.cpp)

# add dependencies
ament_target_dependencies(${PROJECT_NAME} rclcpp)

# expose header file to cpp file
target_include_directories(${PROJECT_NAME} 
                           PUBLIC
                           $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                           $<INSTALL_INTERFACE:include>)

# link Pylon library
target_link_libraries(${PROJECT_NAME} pylon::pylon)

# Expose executable for ROS2 to find
install(TARGETS
       ${PROJECT_NAME}
       DESTINATION lib/${PROJECT_NAME})

ament_package()