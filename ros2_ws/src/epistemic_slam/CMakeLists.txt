cmake_minimum_required(VERSION 3.8)
project(epistemic_slam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# Reconciling two maps and deciding what each robot learned. Deliberately free
# of ROS beyond the message types, so it can be tested against canned grids
# with no graph, no simulator and no robot.
add_library(map_fusion SHARED src/map_fusion.cpp)
target_include_directories(map_fusion PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
target_compile_features(map_fusion PUBLIC cxx_std_17)
ament_target_dependencies(map_fusion nav_msgs geometry_msgs)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS map_fusion
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# The node wires the library to the graph and adds no logic of its own.
add_library(map_fusion_node SHARED src/map_fusion_node.cpp)
target_link_libraries(map_fusion_node map_fusion)
ament_target_dependencies(map_fusion_node rclcpp nav_msgs geometry_msgs std_srvs)

add_executable(map_fusion_main src/map_fusion_main.cpp)
target_link_libraries(map_fusion_main map_fusion_node)
set_target_properties(map_fusion_main PROPERTIES OUTPUT_NAME map_fusion)

install(TARGETS map_fusion_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS map_fusion_main DESTINATION lib/${PROJECT_NAME})

# TT-II, still to come:
#
#   coverage_tracker   Which regions this robot has observed, as opposed to
#                      which cells are free in a map it was handed.
#
#   Announcing the fusion result to the epistemic state. Cells are not atoms,
#   so it needs the region definitions and the translation of thesis 3.7,
#   which lives in eplansys as plansys2_epistemic_perception. That adds a
#   dependency on plansys2_epistemic_msgs, which arrives with eplansys through
#   ros2_ws/eplansys.repos.

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_map_fusion test/test_map_fusion.cpp)
  target_link_libraries(test_map_fusion map_fusion)
endif()

ament_export_include_directories(include)
ament_export_libraries(map_fusion map_fusion_node)
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(rclcpp nav_msgs geometry_msgs std_srvs)

ament_package()
