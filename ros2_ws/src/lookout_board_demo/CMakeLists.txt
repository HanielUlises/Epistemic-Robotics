cmake_minimum_required(VERSION 3.5)
project(lookout_board_demo)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(plansys2_executor REQUIRED)
find_package(plansys2_epistemic_executor REQUIRED)
find_package(plansys2_msgs REQUIRED)

set(dependencies
  rclcpp geometry_msgs nav_msgs sensor_msgs tf2 tf2_ros
  plansys2_executor plansys2_epistemic_executor plansys2_msgs)

add_executable(drive_action_node src/drive_action_node.cpp)
ament_target_dependencies(drive_action_node ${dependencies})

add_executable(sense_action_node src/sense_action_node.cpp)
ament_target_dependencies(sense_action_node ${dependencies})

install(TARGETS drive_action_node sense_action_node
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS scripts/mission.py scripts/recorder.py
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch params config pddl tools
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
  PATTERN "__pycache__" EXCLUDE)

# The EPDDL lives in the repository's epddl-workspace with the other domains,
# written there by tools/make_domain.py, and is installed from there.
install(DIRECTORY
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../epddl-workspace/lookout-board/
  DESTINATION share/${PROJECT_NAME}/epddl
  FILES_MATCHING PATTERN "*.epddl")

ament_package()
