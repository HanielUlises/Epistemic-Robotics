cmake_minimum_required(VERSION 3.8)
project(pass_through_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(epistemic_msgs REQUIRED)
find_package(epistemic_slam REQUIRED)
find_package(mu_path_planner REQUIRED)
# The PlanSys2 targets link Eigen3::Eigen without finding it themselves.
find_package(Eigen3 REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(plansys2_executor REQUIRED)
find_package(plansys2_planner REQUIRED)
find_package(plansys2_problem_expert REQUIRED)
find_package(plansys2_domain_expert REQUIRED)

# The grids, the regions, the safe set and the fixed point. No node in it, so
# the tests exercise it without a graph.
add_library(pass_through_core STATIC src/grid.cpp src/reach.cpp)
target_include_directories(pass_through_core PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
set_target_properties(pass_through_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_link_libraries(pass_through_core
  epistemic_slam::map_fusion
  mu_path_planner::mu_calculus
  mu_path_planner::epistemic_state
  nlohmann_json::nlohmann_json)
ament_target_dependencies(pass_through_core nav_msgs)

add_library(pass_through_driver STATIC src/driver.cpp)
set_target_properties(pass_through_driver PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_link_libraries(pass_through_driver pass_through_core)
ament_target_dependencies(pass_through_driver
  rclcpp nav_msgs sensor_msgs geometry_msgs std_msgs)

add_executable(knowledge_map src/knowledge_map_node.cpp)
target_link_libraries(knowledge_map pass_through_core)
ament_target_dependencies(knowledge_map rclcpp nav_msgs std_msgs epistemic_msgs)

foreach(performer survey_action share_action cross_action)
  add_executable(${performer} src/${performer}_node.cpp)
  target_link_libraries(${performer} pass_through_driver)
  ament_target_dependencies(${performer}
    rclcpp lifecycle_msgs nav_msgs std_msgs epistemic_msgs plansys2_executor)
endforeach()

add_executable(pass_through_mission src/mission_node.cpp)
ament_target_dependencies(pass_through_mission
  rclcpp lifecycle_msgs plansys2_msgs plansys2_executor plansys2_planner
  plansys2_problem_expert plansys2_domain_expert)

install(TARGETS knowledge_map survey_action share_action cross_action pass_through_mission
  DESTINATION lib/${PROJECT_NAME})

# The core and the driver are exported, so a package on the same floor drives
# its robots with the same fixed point instead of a copy of it.
install(DIRECTORY include/ DESTINATION include)
install(TARGETS pass_through_core pass_through_driver
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  INCLUDES DESTINATION include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp nav_msgs sensor_msgs geometry_msgs std_msgs
  nlohmann_json epistemic_slam mu_path_planner)
install(PROGRAMS scripts/knowledge_view.py scripts/camera_director.py
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config params epddl pddl maps tools
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_reach test/test_reach.cpp)
  target_link_libraries(test_reach pass_through_core)
  ament_add_gtest(test_grid test/test_grid.cpp)
  target_link_libraries(test_grid pass_through_core)
endif()

ament_package()
