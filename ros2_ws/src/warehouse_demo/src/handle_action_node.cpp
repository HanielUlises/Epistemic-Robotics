// Copyright 2026 Haniel Vásquez Morales
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// `pick_up` and `drop_off`: a burger has no lift, so this is the one place the
// demonstration stands in for hardware, and it says so rather than pretending.
//
// What matters for the argument is not the lifting. It is that the *planner*
// would not have scheduled a pick_up in a bay the robot does not know the
// pallet to be in -- the modal precondition in the EPDDL domain is what rules
// that out -- and that the branch which reaches this action is the branch the
// observation selected. The dwell is here so the pause is visible; the marker
// is here so a viewer can see which bay it happened in.

#include <memory>
#include <string>

#include "lifecycle_msgs/msg/transition.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "plansys2_executor/ActionExecutorClient.hpp"

class HandleAction : public plansys2::ActionExecutorClient
{
public:
  HandleAction()
  : plansys2::ActionExecutorClient("handle_action")
  {
    declare_parameter<double>("dwell", 4.0);
    declare_parameter<std::string>("verb", "handling");
    marker_ = create_publisher<visualization_msgs::msg::Marker>(
      "/warehouse/handling", rclcpp::QoS(1).transient_local());
  }

private:
  void do_work() override
  {
    if (started_ < 0.0) {
      started_ = now().seconds();
      const auto & args = get_arguments();
      const std::string where = args.size() > 1 ? args[1] : "?";
      RCLCPP_INFO(get_logger(), "%s at %s",
        get_parameter("verb").as_string().c_str(), where.c_str());
      announce(where);
    }

    const double dwell = get_parameter("dwell").as_double();
    const double waited = now().seconds() - started_;
    if (waited >= dwell) {
      started_ = -1.0;
      finish(true, 1.0, get_parameter("verb").as_string() + " done");
      return;
    }
    send_feedback(static_cast<float>(waited / dwell),
      get_parameter("verb").as_string());
  }

  void announce(const std::string & where)
  {
    visualization_msgs::msg::Marker text;
    text.header.frame_id = "map";
    text.header.stamp = now();
    text.ns = "handling";
    text.id = 1;
    text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text.action = visualization_msgs::msg::Marker::ADD;
    text.pose.position.x = 4.0;
    text.pose.position.y = -1.2;
    text.pose.position.z = 0.6;
    text.pose.orientation.w = 1.0;
    text.scale.z = 0.55;
    text.color.r = 0.95; text.color.g = 0.75; text.color.b = 0.2; text.color.a = 1.0;
    text.text = get_parameter("verb").as_string() + " (" + where + ")";
    marker_->publish(text);
  }

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_;
  double started_{-1.0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HandleAction>();
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
