cmake_minimum_required(VERSION 3.8)
project(hotel_rmf_demo)

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

add_executable(hotel_mission src/hotel_mission.cpp)
ament_target_dependencies(hotel_mission
  rclcpp
  plansys2_executor
  plansys2_planner
  plansys2_problem_expert
  plansys2_domain_expert
  plansys2_msgs
)

install(TARGETS hotel_mission DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config params pddl tools
  DESTINATION share/${PROJECT_NAME}
)

# The EPDDL is not copied into this package. It lives in epddl-workspace with
# the other domains, where plank and the planner are run against it directly,
# and a second copy would be a second thing to keep right.
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../../../epddl-workspace/hotel-incident/
  DESTINATION share/${PROJECT_NAME}/epddl
  FILES_MATCHING PATTERN "*.epddl"
)

ament_package()
