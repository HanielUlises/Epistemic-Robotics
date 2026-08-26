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

// Captions for the run, drawn in RViz.
//
// A recording of an epistemic system is hard to read: the robot drives, and
// nothing on screen says which action of the policy that is, whether it is the
// ontic half or the sensing half, or what the mission was asking for. This node
// answers those three questions from what the executor already publishes --
// the policy on /executing_plan, the running action on /action_execution_info
// -- and draws them as text.
//
// It is a spectator. It commands nothing and is not part of the system under
// demonstration; removing it changes only what the video shows.

#include <cmath>
#include <map>
#include <set>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "plansys2_msgs/msg/action_execution_info.hpp"
#include "plansys2_msgs/msg/plan.hpp"

using plansys2_msgs::msg::ActionExecutionInfo;
using plansys2_msgs::msg::Plan;
using plansys2_msgs::msg::PlanItem;

class Caption : public rclcpp::Node
{
public:
  Caption()
  : rclcpp::Node("caption")
  {
    declare_parameter<std::string>("frame", "map");
    declare_parameter<double>("x", 0.0);
    declare_parameter<double>("top", 9.5);

    markers_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/demo/captions", rclcpp::QoS(1).transient_local());

    // Held as members. A subscription whose handle is dropped is destroyed
    // with it, and the node then sits there subscribed to nothing -- which
    // shows up as a caption reading "0 of 0" while the executor is publishing
    // a six-node policy.
    //
    // The executor publishes the plan transient-local with depth 100, so the
    // depth here matches: a smaller queue on a transient-local subscription
    // still works, but matching it keeps the intent obvious.
    plan_sub_ = create_subscription<Plan>(
      "/executing_plan", rclcpp::QoS(100).transient_local(),
      [this](const Plan::SharedPtr msg) {on_plan(msg);});

    action_sub_ = create_subscription<ActionExecutionInfo>(
      "/action_execution_info", 100,
      [this](const ActionExecutionInfo::SharedPtr msg) {on_action(msg);});

    timer_ = create_wall_timer(
      std::chrono::milliseconds(250), [this]() {draw();});
  }

private:
  void on_plan(const Plan::SharedPtr msg)
  {
    // A new plan is a new count. Comparing sizes alone misses a replan that
    // happens to produce as many items, so compare the items themselves.
    bool same = msg->items.size() == plan_.items.size();
    for (std::size_t i = 0; same && i < msg->items.size(); ++i) {
      same = msg->items[i].action == plan_.items[i].action &&
        std::fabs(msg->items[i].time - plan_.items[i].time) < 1e-3;
    }
    if (!same) {
      done_.clear();
      failed_.clear();
    }
    plan_ = *msg;
    // The policy's own vocabulary, keyed by the PDDL expression the executor
    // dispatches, so a running action can be named on both sides at once.
    epistemic_.clear();
    sensing_.clear();
    outcomes_.clear();
    for (std::size_t i = 0; i < plan_.items.size(); ++i) {
      const auto & item = plan_.items[i];
      epistemic_[item.action] = item.epistemic_action;
      sensing_[item.action] = item.sensing;
      index_[item.action] = i;
      std::string outs;
      for (std::size_t k = 0; k < item.outcomes.size(); ++k) {
        outs += (k ? " | " : "") + item.outcomes[k];
      }
      outcomes_[item.action] = outs;
    }
  }

  void on_action(const ActionExecutionInfo::SharedPtr msg)
  {
    const std::string key = msg->action_full_name;
    switch (msg->status) {
      case ActionExecutionInfo::EXECUTING:
        current_ = key;
        completion_ = msg->completion;
        break;
      case ActionExecutionInfo::SUCCEEDED:
        // A set, not a list: the executor republishes an action's final
        // status for as long as the plan runs, so appending counts the same
        // finished action thousands of times -- "3615 of 6 done".
        //
        // Keyed by the full name, time suffix and all. The same expression
        // appears at more than one policy node -- (goto_door r1 door3 door6)
        // sits in both branches -- and only the start time tells them apart,
        // so stripping it would make the count stall short of the policy.
        done_.insert(key);
        if (current_ == key) {current_.clear();}
        break;
      case ActionExecutionInfo::FAILED:
      case ActionExecutionInfo::CANCELLED:
        failed_ = key;
        if (current_ == key) {current_.clear();}
        break;
      default:
        break;
    }
  }

  /// The PDDL expression the executor reports, reduced to the form the plan
  /// carries: "(look_into r1 door3):128001" is item "(look_into r1 door3)".
  static std::string strip(const std::string & full)
  {
    const auto colon = full.rfind(':');
    return colon == std::string::npos ? full : full.substr(0, colon);
  }

  visualization_msgs::msg::Marker text(
    int id, double dy, const std::string & body, double r, double g, double b,
    double size)
  {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = get_parameter("frame").as_string();
    m.header.stamp = now();
    m.ns = "caption";
    m.id = id;
    m.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.position.x = get_parameter("x").as_double();
    m.pose.position.y = get_parameter("top").as_double() + dy;
    m.pose.position.z = 0.0;
    m.pose.orientation.w = 1.0;
    m.scale.z = size;
    m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = 1.0;
    m.text = body;
    return m;
  }

  void draw()
  {
    visualization_msgs::msg::MarkerArray arr;

    const std::string goal = plan_.epistemic_goal.empty() ?
      "waiting for a mission" : plan_.epistemic_goal;
    arr.markers.push_back(
      text(0, 1.4, "GOAL   " + goal, 0.45, 1.0, 0.65, 0.95));

    std::string now_line;
    if (!current_.empty()) {
      const auto item = strip(current_);
      const auto it = epistemic_.find(item);
      const std::string ep = it != epistemic_.end() && !it->second.empty() ?
        it->second : std::string("--");
      const bool is_sensing = sensing_.count(item) && sensing_[item];
      now_line = (is_sensing ? "SENSING  " : "ONTIC    ") + ep + "   " + item +
        "   " + std::to_string(static_cast<int>(completion_ * 100.0)) + "%";
    } else if (!failed_.empty()) {
      now_line = "FAILED   " + strip(failed_);
    } else {
      now_line = "IDLE";
    }
    arr.markers.push_back(text(1, 0.0, now_line, 1.0, 1.0, 1.0, 0.85));

    std::string track = "POLICY   " + std::to_string(done_.size()) + " of " +
      std::to_string(plan_.items.size()) + " done";
    if (!current_.empty()) {
      const auto item = strip(current_);
      if (index_.count(item)) {
        track += "   -   node " + std::to_string(index_[item]);
      }
      if (sensing_.count(item) && sensing_[item] && !outcomes_[item].empty()) {
        track += "   branches on: " + outcomes_[item];
      }
    }
    arr.markers.push_back(text(2, -1.2, track, 1.0, 0.78, 0.30, 0.8));

    markers_->publish(arr);
  }

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_;
  rclcpp::Subscription<Plan>::SharedPtr plan_sub_;
  rclcpp::Subscription<ActionExecutionInfo>::SharedPtr action_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  Plan plan_;
  std::map<std::string, std::string> epistemic_;
  std::map<std::string, bool> sensing_;
  std::map<std::string, std::string> outcomes_;
  std::map<std::string, std::size_t> index_;

  std::string current_;
  std::string failed_;
  std::set<std::string> done_;
  float completion_{0.0f};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Caption>());
  rclcpp::shutdown();
  return 0;
}
