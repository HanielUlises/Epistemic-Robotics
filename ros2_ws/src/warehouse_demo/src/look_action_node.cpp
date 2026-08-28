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
// `look_into`: what inspecting a bay means for a robot that has a laser.
//
// This node deliberately does almost nothing, and that is the point. A sensing
// action in ePlanSys does not report its own outcome: the observation is made
// by plansys2_epistemic_perception, which classifies the occupancy grid over
// the region named for the bay and calls apply_action on the epistemic state
// with the event that fired. If this node decided what was seen, the knowledge
// in the model would come from the executor rather than from the map, and the
// demonstration would be a puppet show.
//
// So it faces the bay, holds the robot there while the region resolves, and
// ends the moment the agent knows -- asking the model, not a stopwatch. Which
// branch of the policy runs next is decided by the outcome perception
// reported, in the epistemic state, where it belongs.

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "plansys2_epistemic_executor/EpistemicStateClient.hpp"
#include "plansys2_executor/ActionExecutorClient.hpp"

class LookAction : public plansys2::ActionExecutorClient
{
public:
  LookAction()
  : plansys2::ActionExecutorClient("look_action")
  {
    // What the agent has to come to know for this action to be done. A sensing
    // action exists to acquire knowledge, so knowledge is what ends it.
    declare_parameter<std::string>("knows_bay2", "(Kw r1 pallet-at_bay2)");
    declare_parameter<std::string>("knows_bay3", "(Kw r1 pallet-at_bay3)");
    declare_parameter<double>("dwell", 40.0);

    // The middle of each bay, in the map frame: what the action looks *at*.
    // The robot turns to face it and then holds, which is what looking into a
    // bay means and what a viewer can read at a glance. A base rotating on the
    // spot says nothing about intent.
    // In the AWS world's frame, which is the map frame. These were the old
    // warehouse's coordinates for a long time, and the symptom was a robot
    // that turned to face a blank stretch of wall and held there until the
    // action timed out -- looking, in the wrong direction, at nothing.
    declare_parameter<std::vector<double>>("face_bay2", {4.50, -2.14});
    declare_parameter<std::vector<double>>("face_bay3", {4.50, -3.94});

    // The robot's own base link. The `map` frame is deliberately *not* a
    // parameter: every robot starts in this world and measures the same
    // building, so they share one coordinate frame and rviz can draw the
    // fleet in it. What differs per robot is the odometry below it, and what
    // each has actually measured -- which is a difference in the maps, not in
    // the frame they are expressed in.
    declare_parameter<std::string>("base_frame", "base_footprint");

    // Which robot is looking. It names this node's client on the epistemic
    // state, and it has to: a client is a node, a node name must be unique,
    // and three look actions sharing one name means two of them lose their
    // rosout publisher and, worse, answer to each other's service replies.
    declare_parameter<std::string>("robot", "r1");

    // Relative, so that launched inside a robot's namespace this drives that
    // robot and nothing else.
    cmd_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    state_ = std::make_shared<plansys2::EpistemicStateClient>(
      "look_action_state_client_" + get_parameter("robot").as_string());
    buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);
  }

private:
  bool pose(double & x, double & y, double & yaw)
  {
    try {
      const auto t = buffer_->lookupTransform(
        "map", get_parameter("base_frame").as_string(), tf2::TimePointZero);
      x = t.transform.translation.x;
      y = t.transform.translation.y;
      const auto & q = t.transform.rotation;
      yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
      return true;
    } catch (const std::exception &) {
      return false;
    }
  }

  void do_work() override
  {
    const auto & args = get_arguments();
    const std::string bay = args.size() > 1 ? args[1] : "bay2";
    if (!has_parameter("face_" + bay)) {
      finish(false, 0.0, "the action names a bay this node cannot face");
      return;
    }
    const auto target = get_parameter("face_" + bay).as_double_array();

    double x = 0.0, y = 0.0, yaw = 0.0;
    if (!pose(x, y, yaw)) {
      send_feedback(0.0, "waiting for a pose");
      return;
    }

    const double bearing = std::atan2(target[1] - y, target[0] - x);
    const double error = std::atan2(std::sin(bearing - yaw), std::cos(bearing - yaw));

    if (started_ < 0.0 && std::fabs(error) > 0.12) {
      // Turning to face what is to be looked at: deliberate, bounded, and it
      // ends pointing into the bay rather than spinning in front of it.
      geometry_msgs::msg::Twist turn;
      turn.angular.z = std::max(-0.9, std::min(0.9, 1.8 * error));
      cmd_->publish(turn);
      send_feedback(0.1f, "turning to face " + bay);
      return;
    }

    if (started_ < 0.0) {
      started_ = now().seconds();
      RCLCPP_INFO(get_logger(), "facing %s, looking", bay.c_str());
    }

    // A slow survey turn, and it ends the moment the agent knows. Standing
    // perfectly still is the cleaner picture, but a mapper integrates little
    // from a base that never moves at all: a bay looked at from a dead stop
    // can stay unobserved, and a region that stays unobserved is one the agent
    // can never come to know. What ends this is the check below, not the
    // motion: the robot turns until it knows, not until a timer runs out.
    geometry_msgs::msg::Twist survey;
    survey.angular.z = 0.3;
    cmd_->publish(survey);

    const double dwell = get_parameter("dwell").as_double();
    const double waited = now().seconds() - started_;
    const std::string knows = get_parameter("knows_" + bay).as_string();

    const auto answer = state_->check_formula(knows);
    if (answer.answered && answer.holds) {
      RCLCPP_INFO(get_logger(), "%s holds after %.1fs", knows.c_str(), waited);
      cmd_->publish(geometry_msgs::msg::Twist{});
      started_ = -1.0;
      finish(true, 1.0, "looked, and now knows");
      return;
    }

    if (waited >= dwell) {
      // The region never settled. Saying so is better than reporting success
      // into an epistemic update that has nothing to apply.
      RCLCPP_WARN(get_logger(), "%s still does not hold after %.0fs",
        knows.c_str(), dwell);
      cmd_->publish(geometry_msgs::msg::Twist{});
      started_ = -1.0;
      finish(false, 1.0, "looked, and learned nothing");
      return;
    }
    send_feedback(static_cast<float>(0.2 + 0.8 * waited / dwell), "looking into " + bay);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_;
  std::shared_ptr<plansys2::EpistemicStateClient> state_;
  std::shared_ptr<tf2_ros::Buffer> buffer_;
  std::shared_ptr<tf2_ros::TransformListener> listener_;
  double started_{-1.0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LookAction>();
  node->set_parameter(rclcpp::Parameter("action_name", "look_into"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
