cmake_minimum_required(VERSION 3.8)
project(mu_path_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(epistemic_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)

# Core mu-calculus library (no ROS deps — testable standalone)
add_library(mu_calculus STATIC
  src/mu_calculus.cpp
)
target_include_directories(mu_calculus PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(mu_calculus PUBLIC cxx_std_17)

# The bridge from the Kripke snapshot on /epistemic/state to the cell sets the
# fixed point runs over. No ROS either: it is tested against snapshot text.
add_library(epistemic_state STATIC
  src/epistemic_state.cpp
)
target_link_libraries(epistemic_state
  mu_calculus
  nlohmann_json::nlohmann_json
)
target_compile_features(epistemic_state PUBLIC cxx_std_17)

# ROS2 planner node
add_executable(mu_path_planner_node
  src/mu_path_planner_node.cpp
)
target_link_libraries(mu_path_planner_node mu_calculus epistemic_state)
ament_target_dependencies(mu_path_planner_node
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
  epistemic_msgs
)

install(TARGETS mu_calculus epistemic_state
  EXPORT mu_calculus_targets
  ARCHIVE DESTINATION lib
)
install(TARGETS mu_path_planner_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)

# Standalone unit test (no ROS required)
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_mu_calculus test/test_mu_calculus.cpp)
  target_link_libraries(test_mu_calculus mu_calculus)

  ament_add_gtest(test_epistemic_state test/test_epistemic_state.cpp)
  target_link_libraries(test_epistemic_state epistemic_state)
endif()

ament_package()
