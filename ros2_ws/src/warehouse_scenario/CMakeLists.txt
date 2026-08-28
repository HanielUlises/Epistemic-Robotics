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
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nlohmann_json REQUIRED)

# The scenario itself: the floor, what the robots know about it, and the
# questions put to the planner. No ROS, so the tests run without a graph.
add_library(warehouse_scenario STATIC src/warehouse.cpp src/floorplan.gen.cpp)
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

# Animates the cases: raw frames to stdout, for ffmpeg to encode.
add_executable(render_video src/render_video.cpp)
target_link_libraries(render_video warehouse_scenario)

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

install(TARGETS make_maps run_offline render_scenario render_video
  scenario_driver
  DESTINATION lib/${PROJECT_NAME}
)
# Exported so the ROS demo beside it can name the same zones rather than
# copying their coordinates into a second file.
install(TARGETS warehouse_scenario
  EXPORT export_warehouse_scenario
  ARCHIVE DESTINATION lib
  INCLUDES DESTINATION include
)
ament_export_targets(export_warehouse_scenario HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(mu_path_planner nlohmann_json)

install(DIRECTORY include/ DESTINATION include)
# The floor plan as map_server reads it, for rviz and for nav2. The library
# carries its own copy; this is the same grid for everything that wants a file.
install(DIRECTORY maps DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_warehouse_scenario test/test_warehouse_scenario.cpp)
  target_link_libraries(test_warehouse_scenario warehouse_scenario)
endif()

ament_package()
