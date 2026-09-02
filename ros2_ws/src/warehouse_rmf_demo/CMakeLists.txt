cmake_minimum_required(VERSION 3.8)
project(warehouse_rmf_demo)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(plansys2_executor REQUIRED)
find_package(plansys2_planner REQUIRED)
find_package(plansys2_problem_expert REQUIRED)
find_package(plansys2_domain_expert REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(warehouse_mission src/warehouse_mission.cpp)
ament_target_dependencies(warehouse_mission
  rclcpp
  plansys2_executor
  plansys2_planner
  plansys2_problem_expert
  plansys2_domain_expert
  plansys2_msgs
)

install(TARGETS warehouse_mission DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config params maps pddl tools
  DESTINATION share/${PROJECT_NAME}
)

# The EPDDL is not copied. It lives in epddl-workspace/robot-warehouse with the
# rest of the domains, exactly as the other demos run it, and a second copy
# would be a second thing to keep right.
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../../../epddl-workspace/robot-warehouse/
  DESTINATION share/${PROJECT_NAME}/epddl
  FILES_MATCHING PATTERN "*.epddl"
)

ament_package()
