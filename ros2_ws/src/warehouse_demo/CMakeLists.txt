cmake_minimum_required(VERSION 3.8)
project(warehouse_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
# plansys2_executor pulls in Eigen through its exported dependencies, and
# ament does not find it for us.
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(epistemic_msgs REQUIRED)
find_package(warehouse_scenario REQUIRED)
find_package(mu_path_planner REQUIRED)
find_package(plansys2_executor REQUIRED)
find_package(plansys2_epistemic_executor REQUIRED)
find_package(plansys2_domain_expert REQUIRED)
find_package(plansys2_problem_expert REQUIRED)
find_package(plansys2_planner REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(plansys2_pddl_parser REQUIRED)

set(action_dependencies
  rclcpp std_msgs nav_msgs sensor_msgs geometry_msgs visualization_msgs
  tf2 tf2_ros epistemic_msgs
  plansys2_executor plansys2_epistemic_executor plansys2_msgs)

# `goto_zone`: asks the mu-calculus planner where it may go, and hands that
# pose to Nav2 to drive to.
add_executable(drive_action_node src/drive_action_node.cpp)
ament_target_dependencies(drive_action_node ${action_dependencies}
  warehouse_scenario mu_path_planner nav2_msgs rclcpp_action)

# `look_into`: faces the bay and waits for the model to say the agent knows.
add_executable(look_action_node src/look_action_node.cpp)
ament_target_dependencies(look_action_node ${action_dependencies})

# `pick_up` and `drop_off`: the one place a burger stands in for a fork lift.
add_executable(handle_action_node src/handle_action_node.cpp)
ament_target_dependencies(handle_action_node ${action_dependencies})

# Draws what ePlanSys knows over the map, for rviz.
add_executable(markers_node src/markers_node.cpp)
ament_target_dependencies(markers_node
  rclcpp std_msgs visualization_msgs plansys2_msgs warehouse_scenario
  tf2 tf2_ros plansys2_epistemic_executor)

# Asks the epistemic state whether each of several formulas holds, twice a
# second, and writes down what it said. An instrument, not a participant: the
# result of the nested run is a moment in a Kripke model, and a model has no
# picture of its own.
add_executable(formula_probe_node src/formula_probe_node.cpp)
ament_target_dependencies(formula_probe_node
  rclcpp std_msgs plansys2_epistemic_executor)

# Seeds the problem, asks for the policy, and hands it to the executor.
add_executable(mission_node src/mission_node.cpp)
ament_target_dependencies(mission_node
  rclcpp plansys2_msgs plansys2_pddl_parser
  plansys2_domain_expert plansys2_problem_expert
  plansys2_planner plansys2_executor)

install(TARGETS drive_action_node look_action_node handle_action_node mission_node
  markers_node formula_probe_node
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch params pddl config worlds
  DESTINATION share/${PROJECT_NAME})

# The EPDDL pair, installed from the workspace where it is maintained: the
# planner and the epistemic state are both given it by path.
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../../../epddl-workspace/robot-warehouse/
  DESTINATION share/${PROJECT_NAME}/epddl
  FILES_MATCHING PATTERN "*.epddl")

# The grounded tasks, which plank made from those very files and which are
# checked in beside them.
#
# The epistemic state is given the grounded task rather than the pair, so it
# does not have to shell out to plank while it is configuring. Grounding the
# three-agent instance is 198 actions and takes a few seconds, and a lifecycle
# node cannot answer `get_state` while it is doing it: its manager times out
# and declares the bringup failed about a second before the grounding finishes,
# with nothing whatsoever wrong with the node.
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../../../epddl-workspace/robot-warehouse/out/
  DESTINATION share/${PROJECT_NAME}/epddl/out
  FILES_MATCHING PATTERN "*.json")

ament_package()
