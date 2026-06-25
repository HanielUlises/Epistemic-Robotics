cmake_minimum_required(VERSION 3.8)
project(epistemic_state)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(epistemic_msgs REQUIRED)

add_executable(epistemic_world_manager
  src/epistemic_world_manager.cpp
)

target_include_directories(epistemic_world_manager PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

ament_target_dependencies(epistemic_world_manager
  rclcpp
  std_msgs
  epistemic_msgs
)

install(TARGETS epistemic_world_manager
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
