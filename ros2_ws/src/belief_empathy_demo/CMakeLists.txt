cmake_minimum_required(VERSION 3.5)
project(belief_empathy_demo)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(plansys2_executor REQUIRED)
find_package(plansys2_msgs REQUIRED)

add_executable(relay_node src/relay_node.cpp)
ament_target_dependencies(relay_node rclcpp std_msgs plansys2_executor plansys2_msgs)

install(TARGETS relay_node
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS scripts/mission.py scripts/recorder.py scripts/ugv_node.py scripts/uav_node.py
  scripts/world_node.py
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch pddl tools policies
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
  OPTIONAL
  PATTERN "__pycache__" EXCLUDE)

# The EPDDL lives in the repository's epddl-workspace with the other domains.
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../../../epddl-workspace/belief-empathy/
  DESTINATION share/${PROJECT_NAME}/epddl FILES_MATCHING PATTERN "*.epddl" PATTERN "out" EXCLUDE)

ament_package()
