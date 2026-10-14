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

// The sensing actions of the lookout board: read_slot, check_view, look_zone.
//
// One executable, one instance per action. Each turns the robot to face what
// the action is about, sweeps a few degrees either side of it so that the
// mapper keeps integrating, and ends on what the robot has come to know, not
// on a clock. What counts as knowing depends on the formulation:
//
//   epistemic   the epistemic state is asked whether the action's formula
//               holds, (Kw r1 clear-k) or (Kw r1 blocked-z). The observation
//               itself is made by plansys2_epistemic_perception from the map;
//               this node never decides an outcome.
//   classical   there is no epistemic state to ask. The node reads the map
//               itself, by the perception's rule, and ends as soon as the
//               region the action is about is decided, or, for a look, as soon
//               as the window in front of it reads blocked. This is the
//               strongest classical look there is: it ends on the same
//               evidence the epistemic one does, and never waits out a
//               declared duration in front of a screen.
//
// A ceiling, `dwell`, ends an action that never settles, as a failure.

#include <algorithm>
#include <cmath>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "plansys2_executor/ActionExecutorClient.hpp"
#include "plansys2_epistemic_executor/EpistemicStateClient.hpp"

namespace
{
constexpr double kFacing = 0.10;      // rad: close enough to start looking
constexpr double kSweep = 0.14;       // rad either side of the target while looking
constexpr double kSweepRate = 0.35;   // rad/s
constexpr int kFreeBelow = 25;        // the perception's thresholds
constexpr int kOccupiedAbove = 65;

enum class Class {Unknown, Clear, Blocked};
const char * name_of(Class c)
{
  return c == Class::Clear ? "clear" : c == Class::Blocked ? "blocked" : "unknown";
}
}  // namespace

class SenseAction : public plansys2::ActionExecutorClient
{
public:
  SenseAction()
  : plansys2::ActionExecutorClient("sense_action")
  {
    declare_parameter<std::string>("mode", "epistemic");
    declare_parameter<double>("dwell", 30.0);
    // Per argument of the action (a slot or a lookout): where to face,
    // [x, y]; what to know, a formula, in the epistemic mode; and, in the
    // classical mode, the region that decides the action, [x0, y0, x1, y1],
    // and for a look the window strip that says it never will.
    declare_parameter<std::vector<std::string>>("targets", std::vector<std::string>{});
    // Copied: as_string_array() refers into the Parameter get_parameter
    // returns, which a range-for over it would outlive.
    const std::vector<std::string> targets = get_parameter("targets").as_string_array();
    for (const auto & t : targets) {
      declare_parameter<std::vector<double>>("face_" + t, std::vector<double>{});
      declare_parameter<std::string>("knows_" + t, "");
      declare_parameter<std::vector<double>>("region_" + t, std::vector<double>{});
      declare_parameter<std::vector<double>>("window_" + t, std::vector<double>{});
    }

    cmd_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);
    if (get_parameter("mode").as_string() == "epistemic") {
      state_ = std::make_shared<plansys2::EpistemicStateClient>(
        get_parameter("action_name").as_string() + "_state_client");
    } else {
      rclcpp::QoS latched(1);
      latched.transient_local().reliable();
      map_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", latched, [this](const nav_msgs::msg::OccupancyGrid::SharedPtr m) {
          std::lock_guard<std::mutex> lock(map_mutex_);
          grid_ = m;
        });
    }
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

  // The perception's rule: blocked if any cell is occupied, clear if every
  // cell is observed and free, unknown otherwise.
  Class classify(const std::vector<double> & box)
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    if (!grid_ || box.size() != 4) {return Class::Unknown;}
    const auto & info = grid_->info;
    const double r = info.resolution;
    bool all_free = true, any = false;
    for (unsigned j = 0; j < info.height; ++j) {
      const double cy = info.origin.position.y + (j + 0.5) * r;
      if (cy < box[1] || cy > box[3]) {continue;}
      for (unsigned i = 0; i < info.width; ++i) {
        const double cx = info.origin.position.x + (i + 0.5) * r;
        if (cx < box[0] || cx > box[2]) {continue;}
        any = true;
        const int v = grid_->data[j * info.width + i];
        if (v > kOccupiedAbove) {return Class::Blocked;}
        if (v < 0 || v >= kFreeBelow) {all_free = false;}
      }
    }
    return any && all_free ? Class::Clear : Class::Unknown;
  }

  void stop() {cmd_->publish(geometry_msgs::msg::Twist());}

  void done(bool ok, const std::string & why)
  {
    stop();
    started_ = -1.0;
    finish(ok, 1.0, why);
  }

  void do_work() override
  {
    const auto & args = get_arguments();
    const std::string target = args.empty() ? "" : args.back();
    const std::string act = get_parameter("action_name").as_string();
    if (!has_parameter("face_" + target) ||
      get_parameter("face_" + target).as_double_array().size() != 2)
    {
      RCLCPP_ERROR(get_logger(), "%s: nothing to face for '%s'", act.c_str(), target.c_str());
      done(false, "the action names nothing this node knows how to face");
      return;
    }
    const auto face = get_parameter("face_" + target).as_double_array();

    double x = 0.0, y = 0.0, yaw = 0.0;
    if (!pose(x, y, yaw)) {
      send_feedback(0.0, "waiting for a pose");
      return;
    }
    const double bearing = std::atan2(face[1] - y, face[0] - x);
    const double error = std::atan2(std::sin(bearing - yaw), std::cos(bearing - yaw));

    if (started_ < 0.0 && std::fabs(error) > kFacing) {
      geometry_msgs::msg::Twist turn;
      turn.angular.z = std::clamp(1.8 * error, -0.9, 0.9);
      cmd_->publish(turn);
      send_feedback(0.1f, "turning to face it");
      return;
    }
    if (started_ < 0.0) {
      started_ = now().seconds();
      sweep_dir_ = 1.0;
      RCLCPP_INFO(get_logger(), "%s %s: facing it, looking", act.c_str(), target.c_str());
    }
    const double waited = now().seconds() - started_;

    // A small sweep either side of the target keeps the mapper integrating
    // without turning the view away from it.
    if (error > kSweep) {sweep_dir_ = 1.0;}
    if (error < -kSweep) {sweep_dir_ = -1.0;}
    geometry_msgs::msg::Twist sweep;
    sweep.angular.z = sweep_dir_ * kSweepRate;
    cmd_->publish(sweep);

    if (state_) {
      const std::string knows = get_parameter("knows_" + target).as_string();
      const auto answer = state_->check_formula(knows);
      if (answer.answered && answer.holds) {
        RCLCPP_INFO(get_logger(), "%s %s: %s holds after %.1fs", act.c_str(), target.c_str(),
          knows.c_str(), waited);
        done(true, "knows");
        return;
      }
    } else {
      const Class region = classify(get_parameter("region_" + target).as_double_array());
      const auto window = get_parameter("window_" + target).as_double_array();
      const Class view = window.size() == 4 ? classify(window) : Class::Unknown;
      if (region != Class::Unknown) {
        RCLCPP_INFO(get_logger(), "%s %s: the map reads it %s after %.1fs", act.c_str(),
          target.c_str(), name_of(region), waited);
        done(true, std::string("decided: ") + name_of(region));
        return;
      }
      if (view == Class::Blocked && waited > 1.0) {
        RCLCPP_INFO(get_logger(), "%s %s: the window reads blocked after %.1fs; nothing to see",
          act.c_str(), target.c_str(), waited);
        done(true, "screened");
        return;
      }
    }

    const double dwell = get_parameter("dwell").as_double();
    if (waited >= dwell) {
      RCLCPP_WARN(get_logger(), "%s %s: nothing settled after %.0fs", act.c_str(),
        target.c_str(), dwell);
      done(false, "looked, and learned nothing");
      return;
    }
    send_feedback(static_cast<float>(0.2 + 0.8 * waited / dwell), "looking");
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_;
  nav_msgs::msg::OccupancyGrid::SharedPtr grid_;
  std::mutex map_mutex_;
  std::shared_ptr<plansys2::EpistemicStateClient> state_;
  std::shared_ptr<tf2_ros::Buffer> buffer_;
  std::shared_ptr<tf2_ros::TransformListener> listener_;
  double started_{-1.0};
  double sweep_dir_{1.0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SenseAction>();
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
