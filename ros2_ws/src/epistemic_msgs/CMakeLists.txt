cmake_minimum_required(VERSION 3.8)
project(epistemic_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# LinkEvent and PartnerBelief are RF-05's: the state of the radio link between
# two robots, and one robot's belief about where its partner is while that link
# is down. They live here and not in epistemic_comm because a message package
# that also builds nodes forces every consumer of the message to build the
# nodes, and the measurement tooling consumes only the messages.
#
# EpistemicEvent is gone. It was consumed by epistemic_state, which eplansys
# replaced: what an action does to a knowledge model now travels as a service
# call to the epistemic state, not as a message on a topic. MuPathQuery stays,
# because mu_path_planner is this repository's own and still uses it.
#
# AbsorbMap is the physical half of a map exchange in pass_through_demo: one
# robot's map handed to another to fuse. It is a service and not a topic
# because an exchange is an event with a result, and the result is what the
# announcement the executor applies has to agree with.
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MuPathQuery.msg"
  "msg/LinkEvent.msg"
  "msg/PartnerBelief.msg"
  "srv/AbsorbMap.srv"
  DEPENDENCIES std_msgs geometry_msgs nav_msgs
)

ament_package()
