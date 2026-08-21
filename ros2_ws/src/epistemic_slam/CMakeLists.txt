cmake_minimum_required(VERSION 3.8)
project(epistemic_slam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(plansys2_epistemic_msgs REQUIRED)

# TT-II. Two nodes, in this order:
#
#   coverage_tracker   Which regions has *this* robot actually observed. It is
#                      not the same question as which cells are marked free in
#                      the map it holds, because a merged map carries cells the
#                      robot never saw. Subscribes to its own map and pose.
#
#   map_fusion         Runs when a link is restored. Reconciles two maps,
#                      decides what each robot now knows, and announces the
#                      difference to the epistemic state. Thesis 3.12.2, CU-03.
#
# Both are testable without a simulator, against canned occupancy grids, which
# is the order to build them in: fusion logic first on fixtures, wiring second.
#
# add_executable(coverage_tracker src/coverage_tracker.cpp)
# ament_target_dependencies(coverage_tracker
#   rclcpp nav_msgs geometry_msgs plansys2_epistemic_msgs)
# install(TARGETS coverage_tracker DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
