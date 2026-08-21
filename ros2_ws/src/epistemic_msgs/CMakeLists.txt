cmake_minimum_required(VERSION 3.8)
project(epistemic_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# EpistemicEvent is gone. It was consumed by epistemic_state, which eplansys
# replaced: what an action does to a knowledge model now travels as a service
# call to the epistemic state, not as a message on a topic. MuPathQuery stays,
# because mu_path_planner is this repository's own and still uses it.
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MuPathQuery.msg"
  DEPENDENCIES std_msgs
)

ament_package()
