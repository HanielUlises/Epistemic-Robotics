cmake_minimum_required(VERSION 3.5)
project(mixed_fleet_demo)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(plansys2_executor REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)

set(dependencies
  rclcpp rclcpp_lifecycle geometry_msgs nav_msgs sensor_msgs std_msgs plansys2_executor plansys2_msgs)

add_executable(drive_node src/drive_node.cpp)
ament_target_dependencies(drive_node ${dependencies})

add_executable(wait_node src/wait_node.cpp)
ament_target_dependencies(wait_node ${dependencies})

# the night porter: a Gazebo actor that walks the routes it is sent
add_library(porter_plugin SHARED src/porter_plugin.cpp)
ament_target_dependencies(porter_plugin rclcpp std_msgs gazebo_dev gazebo_ros)

install(TARGETS drive_node wait_node
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(TARGETS porter_plugin
  LIBRARY DESTINATION lib)

install(PROGRAMS scripts/mission.py scripts/recorder.py scripts/door_node.py scripts/porter_node.py
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch pddl
  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY tools
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
  PATTERN "__pycache__" EXCLUDE)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../../../epddl-workspace/mixed-fleet/
  DESTINATION share/${PROJECT_NAME}/epddl FILES_MATCHING PATTERN "*.epddl")

ament_package()
