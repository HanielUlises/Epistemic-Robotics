cmake_minimum_required(VERSION 3.8)
project(hotel_distributed_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
# The PlanSys2 targets link Eigen3::Eigen without finding it themselves.
find_package(Eigen3 REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(plansys2_executor REQUIRED)
find_package(plansys2_planner REQUIRED)
find_package(plansys2_problem_expert REQUIRED)
find_package(plansys2_domain_expert REQUIRED)
find_package(plansys2_epistemic_msgs REQUIRED)

add_executable(hotel_distributed_mission src/mission_node.cpp)
target_link_libraries(hotel_distributed_mission nlohmann_json::nlohmann_json)
ament_target_dependencies(hotel_distributed_mission
  rclcpp lifecycle_msgs std_msgs plansys2_msgs plansys2_executor plansys2_planner
  plansys2_problem_expert plansys2_domain_expert plansys2_epistemic_msgs)

install(TARGETS hotel_distributed_mission
  DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/hotel_crew.py scripts/knowledge_view.py
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config params pddl tools
  DESTINATION share/${PROJECT_NAME})

# The EPDDL is not copied into this package. tools/hotel_domain.py writes it
# into epddl-workspace with the other domains, where plank and the planner are
# run against it directly.
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../../../epddl-workspace/hotel-distributed/
  DESTINATION share/${PROJECT_NAME}/epddl
  FILES_MATCHING PATTERN "*.epddl"
)

ament_package()
