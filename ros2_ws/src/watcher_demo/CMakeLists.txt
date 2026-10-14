cmake_minimum_required(VERSION 3.5)
project(watcher_demo)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(plansys2_executor REQUIRED)
find_package(plansys2_epistemic_executor REQUIRED)
find_package(plansys2_msgs REQUIRED)

set(dependencies
  rclcpp geometry_msgs nav_msgs sensor_msgs std_msgs
  plansys2_executor plansys2_epistemic_executor plansys2_msgs)

add_executable(drive_node src/drive_node.cpp)
ament_target_dependencies(drive_node ${dependencies})

add_executable(sense_node src/sense_node.cpp)
ament_target_dependencies(sense_node ${dependencies})

add_executable(observer_node src/observer_node.cpp)
ament_target_dependencies(observer_node rclcpp nav_msgs sensor_msgs std_msgs)

install(TARGETS drive_node sense_node observer_node
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS scripts/mission.py scripts/recorder.py
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch pddl tools
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
  PATTERN "__pycache__" EXCLUDE)

# The EPDDL lives in the repository's epddl-workspace with the other domains.
install(DIRECTORY
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../epddl-workspace/watcher/
  DESTINATION share/${PROJECT_NAME}/epddl
  FILES_MATCHING PATTERN "*.epddl")

ament_package()
