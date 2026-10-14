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

// What the carrier reads and shows in the watcher, from its own laser.
//
// One instance per action:
//
//   read_order  before the order board: which of two slots holds the lit post,
//               reported as e-order-x or e-order-y;
//   read_post   before the duty board: which of its two slots holds the lit
//               post, reported as e-peek-west or e-peek-east;
//   look        at a window or a gate approach: whether the watcher stands at
//               the post of that hall, reported as e-look-w-here, e-look-e-clear;
//   signal      at a booth window: the carrier shows its lamp, in the colour of
//               its bay, for `pause` seconds, on /<robot>/lamp, reported as
//               e-say-x-w and so on.
//
// For the two boards the knowledge in the model comes from this laser and from
// nowhere else: a slot counts as lit when it holds at least `min_hits` returns
// on `scans` scans in a row. The lamp is a message, but who sees it is not
// decided here: an observer sees it only while its own laser sees the carrier
// at a window (observer_node).

#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"

#include "plansys2_executor/ActionExecutorClient.hpp"
#include "plansys2_epistemic_executor/EpistemicStateClient.hpp"

class SenseAction : public plansys2::ActionExecutorClient
{
public:
  SenseAction()
  : plansys2::ActionExecutorClient("sense_action")
  {
    declare_parameter<std::string>("robot", "r1");
    declare_parameter<std::string>("mode", "epistemic");
    declare_parameter<std::string>("kind", "read_order");
    // The two slots, [x0, y0, x1, y1] in the world frame, and what each is called.
    declare_parameter<std::vector<double>>("region_a", std::vector<double>{});
    declare_parameter<std::vector<double>>("region_b", std::vector<double>{});
    declare_parameter<std::string>("label_a", "x");
    declare_parameter<std::string>("label_b", "y");
    // The event reported: {v} for the slot's label; for signal, {b} and {s}.
    declare_parameter<std::string>("outcome", "");
    declare_parameter<int>("min_hits", 2);
    declare_parameter<int>("scans", 2);
    declare_parameter<double>("dwell", 30.0);
    declare_parameter<double>("pause", 4.0);
    // look: how long to watch an empty post before calling it clear.
    declare_parameter<double>("look_for", 3.0);

    const std::string robot = get_parameter("robot").as_string();
    if (get_parameter("mode").as_string() == "epistemic") {
      state_ = std::make_shared<plansys2::EpistemicStateClient>(
        get_parameter("action_name").as_string() + "_" + robot + "_state_client");
    }
    scan_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/" + robot + "/scan", rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::LaserScan::SharedPtr m) {on_scan(m);});
    odom_ = create_subscription<nav_msgs::msg::Odometry>(
      "/" + robot + "/odom", 10, [this](const nav_msgs::msg::Odometry::SharedPtr m) {
        std::lock_guard<std::mutex> lock(mutex_);
        px_ = m->pose.pose.position.x;
        py_ = m->pose.pose.position.y;
        const auto & q = m->pose.pose.orientation;
        pyaw_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
        have_pose_ = true;
      });
    cmd_ = create_publisher<geometry_msgs::msg::Twist>("/" + robot + "/cmd_vel", 10);
    lamp_ = create_publisher<std_msgs::msg::String>("/" + robot + "/lamp", 10);
  }

private:
  static bool inside(const std::vector<double> & r, double x, double y)
  {
    return r.size() == 4 && x >= r[0] && x <= r[2] && y >= r[1] && y <= r[3];
  }

  void on_scan(const sensor_msgs::msg::LaserScan::SharedPtr m)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!have_pose_) {return;}
    const auto ra = get_parameter("region_a").as_double_array();
    const auto rb = get_parameter("region_b").as_double_array();
    // The laser sits 0.032 m behind the base on the Burger.
    const double lx = px_ - 0.032 * std::cos(pyaw_), ly = py_ - 0.032 * std::sin(pyaw_);
    int ha = 0, hb = 0;
    for (std::size_t i = 0; i < m->ranges.size(); ++i) {
      const double r = m->ranges[i];
      if (!std::isfinite(r) || r < m->range_min || r > m->range_max) {continue;}
      const double a = pyaw_ + m->angle_min + i * m->angle_increment;
      const double x = lx + r * std::cos(a), y = ly + r * std::sin(a);
      ha += inside(ra, x, y);
      hb += inside(rb, x, y);
    }
    const int need = get_parameter("min_hits").as_int();
    run_a_ = ha >= need ? run_a_ + 1 : 0;
    run_b_ = hb >= need ? run_b_ + 1 : 0;
  }

  static std::string fill(std::string s, const std::string & key, const std::string & value)
  {
    const auto at = s.find(key);
    if (at != std::string::npos) {s.replace(at, key.size(), value);}
    return s;
  }

  // The bay of the order: the action's last argument, or what the robot knows.
  std::string bay()
  {
    const auto & args = get_arguments();
    if (!args.empty() && (args.back() == "bay_x" || args.back() == "bay_y")) {
      return args.back() == "bay_x" ? "x" : "y";
    }
    if (state_) {
      const std::string robot = get_parameter("robot").as_string();
      const auto x = state_->check_formula("(K " + robot + " dest-x)");
      if (x.answered && x.holds) {return "x";}
      const auto kw = state_->check_formula("(Kw " + robot + " dest-x)");
      if (kw.answered && kw.holds) {return "y";}
    }
    return "";
  }

  void done(bool ok, const std::string & why, const std::string & outcome = "")
  {
    cmd_->publish(geometry_msgs::msg::Twist());
    started_ = -1.0;
    finish(ok, 1.0, why, outcome);
  }

  void do_work() override
  {
    const std::string kind = get_parameter("kind").as_string();
    const std::string act = get_parameter("action_name").as_string();
    const std::string robot = get_parameter("robot").as_string();
    const std::string outcome = get_parameter("outcome").as_string();
    const double t = now().seconds();
    if (started_ < 0.0) {
      started_ = t;
      std::lock_guard<std::mutex> lock(mutex_);
      run_a_ = run_b_ = 0;
      lamp_bay_ = kind == "signal" ? bay() : "";
      RCLCPP_INFO(get_logger(), "%s %s: started", act.c_str(), robot.c_str());
    }
    const double waited = t - started_;

    if (kind == "signal") {
      if (lamp_bay_.empty()) {
        RCLCPP_ERROR(get_logger(), "%s %s: the robot does not know its bay", act.c_str(), robot.c_str());
        done(false, "no bay");
        return;
      }
      std_msgs::msg::String msg;
      if (waited < get_parameter("pause").as_double()) {
        msg.data = lamp_bay_;
        lamp_->publish(msg);
        send_feedback(static_cast<float>(waited / get_parameter("pause").as_double()), "showing the lamp");
        return;
      }
      msg.data = "";
      lamp_->publish(msg);
      double x;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        x = px_;
      }
      const std::string side = x < 0.0 ? "w" : "e";
      RCLCPP_INFO(get_logger(), "%s %s: showed bay %s at the %s window", act.c_str(), robot.c_str(),
        lamp_bay_.c_str(), side.c_str());
      done(true, "shown", fill(fill(outcome, "{b}", lamp_bay_), "{s}", side));
      return;
    }

    if (kind == "look") {
      // The post of the hall the carrier stands in: region_a west, region_b east.
      double x;
      int ra, rb;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        x = px_;
        ra = run_a_;
        rb = run_b_;
      }
      const std::string side = x < 0.0 ? "w" : "e";
      const int run = side == "w" ? ra : rb;
      if (run >= get_parameter("scans").as_int()) {
        RCLCPP_INFO(get_logger(), "%s %s: the watcher is at the %s post", act.c_str(), robot.c_str(), side.c_str());
        done(true, "here", fill(fill(outcome, "{s}", side), "{v}", "here"));
        return;
      }
      if (waited >= get_parameter("look_for").as_double()) {
        RCLCPP_INFO(get_logger(), "%s %s: the %s post is empty", act.c_str(), robot.c_str(), side.c_str());
        done(true, "clear", fill(fill(outcome, "{s}", side), "{v}", "clear"));
        return;
      }
      send_feedback(static_cast<float>(waited / get_parameter("look_for").as_double()), "looking");
      return;
    }

    // read_order and read_post: which of the two slots holds the lit post.
    int ra, rb;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      ra = run_a_;
      rb = run_b_;
    }
    const int need = get_parameter("scans").as_int();
    if (ra >= need || rb >= need) {
      const std::string v = get_parameter(ra >= need ? "label_a" : "label_b").as_string();
      RCLCPP_INFO(get_logger(), "%s %s: slot %s is lit, after %.1fs", act.c_str(), robot.c_str(), v.c_str(),
        waited);
      done(true, v, fill(outcome, "{v}", v));
      return;
    }
    if (waited >= get_parameter("dwell").as_double()) {
      RCLCPP_WARN(get_logger(), "%s %s: no slot lit in %.0fs", act.c_str(), robot.c_str(), waited);
      done(false, "nothing seen");
      return;
    }
    send_feedback(static_cast<float>(waited / get_parameter("dwell").as_double()), "reading");
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr lamp_;
  std::shared_ptr<plansys2::EpistemicStateClient> state_;
  std::mutex mutex_;
  bool have_pose_{false};
  double px_{0.0}, py_{0.0}, pyaw_{0.0};
  int run_a_{0}, run_b_{0};
  double started_{-1.0};
  std::string lamp_bay_;
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
