cmake_minimum_required(VERSION 3.5)
project(collapsed_wing_demo)

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

add_executable(wait_node src/wait_node.cpp)
ament_target_dependencies(wait_node ${dependencies})

install(TARGETS drive_node wait_node
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS scripts/mission.py scripts/recorder.py scripts/doorway_node.py
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch pddl tools
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
  PATTERN "__pycache__" EXCLUDE)

# The EPDDL lives in the repository's epddl-workspace with the other domains.
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../../../epddl-workspace/collapsed-wing/
  DESTINATION share/${PROJECT_NAME}/epddl FILES_MATCHING PATTERN "*.epddl" PATTERN "out" EXCLUDE)

ament_package()
