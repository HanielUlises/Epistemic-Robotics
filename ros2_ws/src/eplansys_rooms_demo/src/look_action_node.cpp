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

// The sensing half: what `inspect3` and `inspect6` mean for a robot that has a
// laser.
//
// This node deliberately does almost nothing, and the reason is the point of
// the demo. A sensing action in ePlanSys does not report its own outcome: the
// observation is made by plansys2_epistemic_perception, which classifies the
// occupancy grid over the region named for the room and calls apply_action on
// the epistemic state with the event that fired. If this node decided what was
// seen, the knowledge in the model would come from the executor rather than
// from the map, and the demo would be a puppet show.
//
// So all it does is hold the robot still long enough for the region to be
// resolved -- SLAM needs a moment at the door, and perception reports on
// change rather than on every grid -- and then report that the looking itself
// is done. Which branch of the policy runs next is decided by the outcome
// perception reported, in the epistemic state, where it belongs.

#include <memory>
#include <string>

#include <cmath>
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
    // How long to stand still and look. Long enough for a scan to sweep the
    // room through the doorway and for the grid to settle; short enough that
    // the run does not stall on it.
    // What the agent has to come to know for this action to be done. A
    // sensing action exists to acquire knowledge, so it ends when the
    // knowledge is there -- not when a stopwatch runs out. The dwell below is
    // only a ceiling: if the region never resolves, the action fails and says
    // so, rather than reporting success into an update that cannot happen.
    declare_parameter<std::string>("knows_door3", "(Kw r1 blocked-r3)");
    declare_parameter<std::string>("knows_door6", "(Kw r1 blocked-r6)");
    declare_parameter<double>("dwell", 25.0);

    // Where to stand to see into each room, as [x, y] in the map frame.
    //
    // The sensing action positions the sensor and then looks, which is what
    // makes it a sensing action rather than a pause. It also settles an
    // ordering the demonstration cannot otherwise win: perception reports the
    // moment a region resolves, and the epistemic state will not accept an
    // observation from a robot it does not yet believe is at the door. The
    // drive therefore stops in the corridor, its epistemic update lands, and
    // only then does this action cross the threshold and bring the room into
    // view.
    // What the action looks *at*: the middle of the region whose class it is
    // dispatched to settle. The robot turns to face it and then holds, which
    // is what looking into a room means, and what a viewer can read at a
    // glance. A base rotating on the spot says nothing about intent.
    declare_parameter<std::vector<double>>("face_door3", {6.8, 1.9});
    declare_parameter<std::vector<double>>("face_door6", {4.6, -1.1});

    cmd_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    state_ = std::make_shared<plansys2::EpistemicStateClient>("look_action_state_client");
    buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);
  }

private:
  bool pose(double & x, double & y, double & yaw)
  {
    try {
      const auto t = buffer_->lookupTransform("map", "base_footprint", tf2::TimePointZero);
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
    const std::string name = args.empty() ? "face_door3" : "face_" + args.back();
    if (!has_parameter(name)) {
      finish(false, 0.0, "the action names nothing this node knows how to face");
      return;
    }
    const auto target = get_parameter(name).as_double_array();

    double x = 0.0, y = 0.0, yaw = 0.0;
    if (!pose(x, y, yaw)) {
      send_feedback(0.0, "waiting for a pose");
      return;
    }

    const double bearing = std::atan2(target[1] - y, target[0] - x);
    const double error = std::atan2(std::sin(bearing - yaw), std::cos(bearing - yaw));

    if (started_ < 0.0 && std::fabs(error) > 0.12) {
      // Turning to face what is to be looked at: deliberate, bounded, and it
      // ends pointing into the room rather than spinning in it.
      geometry_msgs::msg::Twist turn;
      turn.angular.z = std::max(-0.9, std::min(0.9, 1.8 * error));
      cmd_->publish(turn);
      send_feedback(0.1f, "turning to face the room");
      return;
    }

    if (started_ < 0.0) {
      started_ = now().seconds();
      RCLCPP_INFO(
        get_logger(), "%s: facing the room, looking",
        get_parameter("action_name").as_string().c_str());
    }

    // A slow survey turn, and it ends the moment the agent knows.
    //
    // Standing perfectly still is the cleaner picture, but the mapper
    // integrates almost nothing from a base that never moves -- a room looked
    // at from a standstill stays unobserved, and a region that stays
    // unobserved is one the agent can never come to know. Turning slowly
    // brings the room through the beam and keeps the mapper working.
    //
    // The important part is what ends it: the check below, not this motion.
    // The robot turns until it knows, which is usually well under one
    // revolution, rather than spinning out a timer.
    geometry_msgs::msg::Twist survey;
    survey.angular.z = 0.35;
    cmd_->publish(survey);

    const double dwell = get_parameter("dwell").as_double();
    const double waited = now().seconds() - started_;

    // Ask the model, not the clock. Perception reports what the map settled
    // to and the state applies it; the moment that has happened the agent
    // knows whether the room is blocked, and the looking is over.
    const std::string knows =
      get_parameter(args.empty() ? "knows_door3" : "knows_" + args.back()).as_string();
    const auto answer = state_->check_formula(knows);
    if (answer.answered && answer.holds) {
      RCLCPP_INFO(
        get_logger(), "%s: %s holds after %.1fs", name.c_str(), knows.c_str(), waited);
      started_ = -1.0;
      finish(true, 1.0, "looked, and now knows");
      return;
    }

    if (waited >= dwell) {
      // The region never settled. Saying so is better than reporting success
      // into an epistemic update that has nothing to apply.
      RCLCPP_WARN(
        get_logger(), "%s: %s still does not hold after %.0fs",
        name.c_str(), knows.c_str(), dwell);
      started_ = -1.0;
      finish(false, 1.0, "looked, and learned nothing");
      return;
    }
    send_feedback(static_cast<float>(0.2 + 0.8 * waited / dwell), "looking");
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
  // action_name comes from the launch file: one instance per inspect action.
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
