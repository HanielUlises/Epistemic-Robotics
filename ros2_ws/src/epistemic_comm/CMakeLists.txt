cmake_minimum_required(VERSION 3.8)
project(epistemic_comm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(epistemic_msgs REQUIRED)

set(dependencies rclcpp std_msgs std_srvs geometry_msgs nav_msgs sensor_msgs
    visualization_msgs epistemic_msgs)

# The propagation is a library and not a node's private detail, because the
# acceptance criterion of RF-05 is a claim about this arithmetic and a claim
# has to be testable without a simulator.
add_library(belief_propagation src/belief_propagation.cpp)
target_include_directories(belief_propagation PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(comm_monitor src/comm_monitor_node.cpp)
ament_target_dependencies(comm_monitor ${dependencies})

add_executable(link_gate src/link_gate_node.cpp)
ament_target_dependencies(link_gate ${dependencies})

add_executable(belief_update src/belief_update_node.cpp)
ament_target_dependencies(belief_update ${dependencies})
target_link_libraries(belief_update belief_propagation)

add_executable(patrol src/patrol_node.cpp)
ament_target_dependencies(patrol ${dependencies})

add_executable(link_experiment src/link_experiment_node.cpp)
ament_target_dependencies(link_experiment ${dependencies})

install(TARGETS belief_propagation
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS comm_monitor link_gate belief_update patrol link_experiment
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS scripts/reconcile.py scripts/grid_align.py
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch params config tools DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_belief_propagation test/test_belief_propagation.cpp)
  target_link_libraries(test_belief_propagation belief_propagation)
endif()

ament_export_include_directories(include)
ament_export_libraries(belief_propagation)
ament_export_dependencies(${dependencies})
ament_package()
