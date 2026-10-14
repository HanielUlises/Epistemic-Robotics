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

// The robot's own observations in the gate signal, from its own laser.
//
// One instance per action and robot. Each reads the robot's scan, places its
// returns in the world with the robot's odometry, and counts the returns that
// fall inside the regions the action is about:
//
//   read_order  r1 before its order board: which slot holds the post;
//   watch       r2 in its booth: which gate has r1 standing in it;
//   look_bay    r2 at a bay's north door: whether anything stands in the bay;
//   unload      either robot: a short pause, reporting the bay it stands in.
//
// What it saw is reported as the outcome the action's model names, e-order-x,
// e-seen-y and so on, and the executor applies that event: the knowledge in the
// model comes from this laser, and from nowhere else. A region counts as seen
// when it holds at least `min_hits` returns on `scans` scans in a row.

#include <cmath>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "plansys2_executor/ActionExecutorClient.hpp"

class SenseAction : public plansys2::ActionExecutorClient
{
public:
  SenseAction()
  : plansys2::ActionExecutorClient("sense_action")
  {
    declare_parameter<std::string>("robot", "r1");
    // kind: read_order, watch, look_bay or unload
    declare_parameter<std::string>("kind", "watch");
    // Regions, [x0, y0, x1, y1] in the world frame, per bay.
    declare_parameter<std::vector<double>>("region_x", std::vector<double>{});
    declare_parameter<std::vector<double>>("region_y", std::vector<double>{});
    // The event reported, with {b} for the bay; for look_bay, {s} for what was seen.
    declare_parameter<std::string>("outcome", "");
    declare_parameter<int>("min_hits", 2);
    declare_parameter<int>("scans", 2);
    declare_parameter<double>("dwell", 60.0);
    declare_parameter<double>("pause", 2.0);
    declare_parameter<double>("look_for", 3.0);

    const std::string robot = get_parameter("robot").as_string();
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
    const auto rx = get_parameter("region_x").as_double_array();
    const auto ry = get_parameter("region_y").as_double_array();
    // The laser sits 0.032 m behind the base on the Burger.
    const double lx = px_ - 0.032 * std::cos(pyaw_), ly = py_ - 0.032 * std::sin(pyaw_);
    int hx = 0, hy = 0;
    for (std::size_t i = 0; i < m->ranges.size(); ++i) {
      const double r = m->ranges[i];
      if (!std::isfinite(r) || r < m->range_min || r > m->range_max) {continue;}
      const double a = pyaw_ + m->angle_min + i * m->angle_increment;
      const double x = lx + r * std::cos(a), y = ly + r * std::sin(a);
      hx += inside(rx, x, y);
      hy += inside(ry, x, y);
    }
    const int need = get_parameter("min_hits").as_int();
    run_x_ = hx >= need ? run_x_ + 1 : 0;
    run_y_ = hy >= need ? run_y_ + 1 : 0;
    last_x_ = hx;
    last_y_ = hy;
  }

  std::string bay_here()
  {
    const auto rx = get_parameter("region_x").as_double_array();
    const auto ry = get_parameter("region_y").as_double_array();
    std::lock_guard<std::mutex> lock(mutex_);
    if (inside(rx, px_, py_)) {return "x";}
    if (inside(ry, px_, py_)) {return "y";}
    return "";
  }

  void done(bool ok, const std::string & why, const std::string & outcome = "")
  {
    cmd_->publish(geometry_msgs::msg::Twist());
    started_ = -1.0;
    finish(ok, 1.0, why, outcome);
  }

  std::string fill(std::string s, const std::string & key, const std::string & value)
  {
    const auto at = s.find(key);
    if (at != std::string::npos) {s.replace(at, key.size(), value);}
    return s;
  }

  void do_work() override
  {
    const std::string kind = get_parameter("kind").as_string();
    const std::string act = get_parameter("action_name").as_string();
    const std::string robot = get_parameter("robot").as_string();
    const double t = now().seconds();
    if (started_ < 0.0) {
      started_ = t;
      std::lock_guard<std::mutex> lock(mutex_);
      run_x_ = run_y_ = 0;
      RCLCPP_INFO(get_logger(), "%s %s: looking", act.c_str(), robot.c_str());
    }
    const double waited = t - started_;
    const int need = get_parameter("scans").as_int();
    const std::string outcome = get_parameter("outcome").as_string();

    if (kind == "unload") {
      if (waited < get_parameter("pause").as_double()) {
        send_feedback(static_cast<float>(waited / get_parameter("pause").as_double()), "unloading");
        return;
      }
      const std::string b = bay_here();
      RCLCPP_INFO(get_logger(), "%s %s: unloaded in bay %s", act.c_str(), robot.c_str(), b.c_str());
      done(true, "unloaded", outcome.empty() || b.empty() ? "" : fill(outcome, "{b}", b));
      return;
    }

    int rx, ry, lx, ly;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      rx = run_x_;
      ry = run_y_;
      lx = last_x_;
      ly = last_y_;
    }
    if (kind == "look_bay") {
      // The bay named by the action's last argument: anything in it?
      const auto & args = get_arguments();
      const std::string b = !args.empty() && args.back() == "bay_y" ? "y" : "x";
      const int run = b == "x" ? rx : ry;
      if (run >= need) {
        RCLCPP_INFO(get_logger(), "%s %s: bay %s is occupied (%d returns)", act.c_str(), robot.c_str(),
          b.c_str(), b == "x" ? lx : ly);
        done(true, "occupied", fill(fill(outcome, "{b}", b), "{s}", "occupied"));
        return;
      }
      if (waited >= get_parameter("look_for").as_double()) {
        RCLCPP_INFO(get_logger(), "%s %s: bay %s is empty", act.c_str(), robot.c_str(), b.c_str());
        done(true, "empty", fill(fill(outcome, "{b}", b), "{s}", "empty"));
        return;
      }
      send_feedback(static_cast<float>(waited / get_parameter("look_for").as_double()), "looking");
      return;
    }
    // read_order and watch: which of the two regions holds something.
    if (rx >= need || ry >= need) {
      const std::string b = rx >= need ? "x" : "y";
      RCLCPP_INFO(get_logger(), "%s %s: region %s holds %d returns after %.1fs", act.c_str(),
        robot.c_str(), b.c_str(), b == "x" ? lx : ly, waited);
      done(true, "seen " + b, fill(outcome, "{b}", b));
      return;
    }
    if (waited >= get_parameter("dwell").as_double()) {
      RCLCPP_WARN(get_logger(), "%s %s: nothing seen in %.0fs", act.c_str(), robot.c_str(), waited);
      done(false, "nothing seen");
      return;
    }
    send_feedback(static_cast<float>(waited / get_parameter("dwell").as_double()), "looking");
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_;
  std::mutex mutex_;
  bool have_pose_{false};
  double px_{0.0}, py_{0.0}, pyaw_{0.0};
  int run_x_{0}, run_y_{0}, last_x_{0}, last_y_{0};
  double started_{-1.0};
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
