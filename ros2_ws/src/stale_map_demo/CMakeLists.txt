cmake_minimum_required(VERSION 3.8)
project(stale_map_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(epistemic_msgs REQUIRED)
find_package(epistemic_slam REQUIRED)
find_package(pass_through_demo REQUIRED)
# The PlanSys2 targets link Eigen3::Eigen without finding it themselves.
find_package(Eigen3 REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(plansys2_executor REQUIRED)
find_package(plansys2_planner REQUIRED)
find_package(plansys2_problem_expert REQUIRED)
find_package(plansys2_domain_expert REQUIRED)
find_package(plansys2_epistemic_msgs REQUIRED)

# One per robot: the shift map, kept current by the robot's laser and by the
# maps it is sent.
add_executable(living_map src/living_map_node.cpp)
target_link_libraries(living_map pass_through_demo::pass_through_core nlohmann_json::nlohmann_json)
ament_target_dependencies(living_map rclcpp nav_msgs sensor_msgs std_msgs epistemic_msgs epistemic_slam)

# What happens on the floor, for every fleet: the forklift, the exchanges and
# the hauls. The haulers drive with pass_through_demo's driver.
add_executable(crew src/crew_node.cpp)
target_link_libraries(crew pass_through_demo::pass_through_driver nlohmann_json::nlohmann_json)
ament_target_dependencies(crew rclcpp nav_msgs std_msgs gazebo_msgs epistemic_msgs)

# The epistemic fleet's performers, each relaying its action to the crew.
add_executable(relay_action src/relay_action_node.cpp)
target_link_libraries(relay_action nlohmann_json::nlohmann_json)
ament_target_dependencies(relay_action rclcpp lifecycle_msgs std_msgs plansys2_executor)

add_executable(stale_maps_mission src/mission_node.cpp)
target_link_libraries(stale_maps_mission nlohmann_json::nlohmann_json)
ament_target_dependencies(stale_maps_mission
  rclcpp lifecycle_msgs std_msgs plansys2_msgs plansys2_executor plansys2_planner
  plansys2_problem_expert plansys2_domain_expert plansys2_epistemic_msgs)

install(TARGETS living_map crew relay_action stale_maps_mission
  DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/knowledge_view.py scripts/camera_director.py
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config params epddl pddl maps tools
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE)

ament_package()
