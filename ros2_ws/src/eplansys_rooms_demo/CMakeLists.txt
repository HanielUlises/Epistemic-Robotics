cmake_minimum_required(VERSION 3.5)
project(eplansys_rooms_demo)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(plansys2_executor REQUIRED)
find_package(plansys2_epistemic_executor REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

set(dependencies
  rclcpp geometry_msgs nav_msgs sensor_msgs
  tf2 tf2_ros tf2_geometry_msgs
  plansys2_executor plansys2_epistemic_executor plansys2_msgs)

add_executable(drive_action_node src/drive_action_node.cpp)
ament_target_dependencies(drive_action_node ${dependencies})

add_executable(look_action_node src/look_action_node.cpp)
ament_target_dependencies(look_action_node ${dependencies})

add_executable(caption_node src/caption_node.cpp)
ament_target_dependencies(caption_node rclcpp visualization_msgs plansys2_msgs)

install(TARGETS drive_action_node look_action_node caption_node
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch params config worlds pddl
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL)

# The EPDDL specification lives in the repository's epddl-workspace, with the
# other domains, rather than being copied in here. Installing it into the
# package's share makes the launch file self-contained without giving the
# domain a second home to drift from.
install(DIRECTORY
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../epddl-workspace/building-rooms/
  DESTINATION share/${PROJECT_NAME}/epddl
  FILES_MATCHING PATTERN "*.epddl")

ament_package()
