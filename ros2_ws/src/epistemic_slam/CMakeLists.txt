cmake_minimum_required(VERSION 3.8)
project(epistemic_slam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(epistemic_msgs REQUIRED)

# TODO (TT-II): add slam_bridge_node once SLAM Toolbox integration begins
# add_executable(slam_bridge src/slam_bridge.cpp)
# ament_target_dependencies(slam_bridge rclcpp sensor_msgs nav_msgs epistemic_msgs)
# install(TARGETS slam_bridge DESTINATION lib/${PROJECT_NAME})

ament_package()
