cmake_minimum_required(VERSION 3.8)
project(muddy_robots_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
# The driver and its routes are pass_through_demo's, over this floor's plan.
find_package(pass_through_demo REQUIRED)
# The PlanSys2 targets link Eigen3::Eigen without finding it themselves.
find_package(Eigen3 REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(plansys2_executor REQUIRED)
find_package(plansys2_planner REQUIRED)
find_package(plansys2_problem_expert REQUIRED)
find_package(plansys2_domain_expert REQUIRED)
find_package(plansys2_epistemic_msgs REQUIRED)

add_executable(muster_crew src/muster_crew_node.cpp)
target_link_libraries(muster_crew pass_through_demo::pass_through_driver
  nlohmann_json::nlohmann_json)
ament_target_dependencies(muster_crew
  rclcpp lifecycle_msgs nav_msgs std_msgs gazebo_msgs plansys2_executor)

add_executable(muddy_robots_mission src/mission_node.cpp)
target_link_libraries(muddy_robots_mission nlohmann_json::nlohmann_json)
ament_target_dependencies(muddy_robots_mission
  rclcpp lifecycle_msgs std_msgs plansys2_msgs plansys2_executor plansys2_planner
  plansys2_problem_expert plansys2_domain_expert plansys2_epistemic_msgs)

install(TARGETS muster_crew muddy_robots_mission
  DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/knowledge_view.py
  DESTINATION lib/${PROJECT_NAME})
# No epddl/ or pddl/: tools/muster.py writes both at launch, for the number of
# robots asked for.
install(DIRECTORY launch config params tools docs
  DESTINATION share/${PROJECT_NAME})

ament_package()
