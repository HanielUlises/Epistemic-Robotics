cmake_minimum_required(VERSION 3.8)
project(coordinated_attack_demo)

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
# The driver and the fixed point are pass_through_demo's: the same floor, the
# same routes.
find_package(pass_through_demo REQUIRED)
# The PlanSys2 targets link Eigen3::Eigen without finding it themselves.
find_package(Eigen3 REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(plansys2_executor REQUIRED)
find_package(plansys2_planner REQUIRED)
find_package(plansys2_problem_expert REQUIRED)
find_package(plansys2_domain_expert REQUIRED)
find_package(plansys2_epistemic_msgs REQUIRED)

include_directories(include)

foreach(performer read_order_action go_view_action radio_action signal_action lift_action)
  add_executable(${performer} src/${performer}_node.cpp)
  target_link_libraries(${performer} pass_through_demo::pass_through_driver)
  ament_target_dependencies(${performer}
    rclcpp lifecycle_msgs nav_msgs std_msgs gazebo_msgs plansys2_executor)
endforeach()

add_executable(coordinated_attack_mission src/mission_node.cpp)
target_link_libraries(coordinated_attack_mission nlohmann_json::nlohmann_json)
ament_target_dependencies(coordinated_attack_mission
  rclcpp lifecycle_msgs plansys2_msgs plansys2_executor plansys2_planner
  plansys2_problem_expert plansys2_domain_expert plansys2_epistemic_msgs)

install(TARGETS read_order_action go_view_action radio_action signal_action lift_action
  coordinated_attack_mission
  DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/knowledge_view.py scripts/camera_director.py
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config params epddl pddl maps tools
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_sight test/test_sight.cpp)
  target_link_libraries(test_sight pass_through_demo::pass_through_core)
endif()

ament_package()
