cmake_minimum_required(VERSION 3.8)
project(warehouse_scenario)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(epistemic_msgs REQUIRED)
find_package(mu_path_planner REQUIRED)
find_package(nlohmann_json REQUIRED)

# The scenario itself: the floor, what the robots know about it, and the
# questions put to the planner. No ROS, so the tests run without a graph.
add_library(warehouse_scenario STATIC src/warehouse.cpp)
target_include_directories(warehouse_scenario PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(warehouse_scenario PUBLIC cxx_std_17)
target_link_libraries(warehouse_scenario
  mu_path_planner::epistemic_state
  mu_path_planner::mu_calculus
  nlohmann_json::nlohmann_json
)

# Writes the two stages and the three snapshots to disk.
add_executable(make_maps src/make_maps.cpp)
target_link_libraries(make_maps warehouse_scenario)

# Answers every case directly, with no ROS in the way.
add_executable(run_offline src/run_offline.cpp)
target_include_directories(run_offline PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(run_offline warehouse_scenario nlohmann_json::nlohmann_json)

# Draws the answers over the floor they were computed on.
add_executable(render_scenario src/render_scenario.cpp)
target_link_libraries(render_scenario warehouse_scenario nlohmann_json::nlohmann_json)

# Puts the same cases to a running planner node over topics.
add_executable(scenario_driver src/scenario_driver.cpp)
target_include_directories(scenario_driver PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(scenario_driver warehouse_scenario nlohmann_json::nlohmann_json)
ament_target_dependencies(scenario_driver
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
  epistemic_msgs
)

install(TARGETS make_maps run_offline render_scenario scenario_driver
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_warehouse_scenario test/test_warehouse_scenario.cpp)
  target_link_libraries(test_warehouse_scenario warehouse_scenario)
endif()

ament_package()
