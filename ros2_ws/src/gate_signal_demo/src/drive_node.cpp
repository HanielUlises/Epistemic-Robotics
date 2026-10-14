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

// The drives of the gate signal, for one robot.
//
// One instance per drive action and robot: to_gate, back_route, enter_bay and
// go, each taking its route from parameters written by the launch from
// tools/layout.py, as a list of roadmap places per bay. The controller is the
// dynamic window of eplansys_rooms_demo's drive node, unchanged. What is new:
//
//  - the pose comes from the robot's own odometry, which the simulator reports
//    in the world frame, so two robots need no shared transform tree;
//  - which bay an action concerns is its last argument when it names one, as
//    the classical domain's actions do; when it does not, as in the epistemic
//    domain's (to_gate r1), it is the bay the robot knows its order names, asked
//    of the epistemic state, or for enter_bay the gate the robot stands in;
//  - an action whose epistemic counterpart has one event per bay reports which
//    occurred, e-gate-x or e-gate-y, so that the executor applies the event the
//    world chose.

#include <algorithm>
#include <cmath>
#include <map>
#include <set>
#include <sstream>
#include <mutex>
#include <utility>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "rclcpp/rclcpp.hpp"

#include "lifecycle_msgs/msg/transition.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "plansys2_executor/ActionExecutorClient.hpp"
#include "plansys2_epistemic_executor/EpistemicStateClient.hpp"

namespace
{
// The controller these numbers belong to is a dynamic window, ported from the
// one validated in simulation for this building: sample the velocities one
// acceleration step away, roll each forward as a constant-curvature arc, throw
// out any arc that brings the footprint within COLLISION of a laser return,
// and score what survives on progress, clearance and speed.
//
// A stop guard alone is not obstacle avoidance. It stops -- and a base that
// stops in front of a crate in room 1 has not avoided anything, it has parked.

constexpr double kRobotRadius = 0.20;      // a burger is 0.178 wide; the rest is
                                           // margin, because every scrape puts drift
                                           // into the map the demo reads knowledge from
constexpr double kCollision = kRobotRadius + 0.02;

constexpr double kVMax = 0.22;             // m/s: a burger tops out near 0.22, and
                                           // speed here buys nothing but collisions
constexpr double kWMax = 1.6;              // rad/s
constexpr double kAccelV = 0.7;            // m/s^2
constexpr double kAccelW = 2.0;            // rad/s^2
constexpr double kControlDt = 0.1;         // s, one tick
constexpr double kHorizon = 1.5;           // s, how far an arc is rolled
constexpr int kSimSteps = 14;
constexpr int kNv = 7;
constexpr int kNw = 25;

// The three terms pull against each other. Clearance is capped: past
// kClearCap metres, more room is not worth trading speed for.
constexpr double kWGoal = 1.0;
constexpr double kWClear = 0.60;
constexpr double kWSpeed = 0.50;
constexpr double kClearCap = 1.0;

constexpr double kLookahead = 1.1;         // m along the route
constexpr double kArrived = 0.35;          // m, a leg counts as reached
constexpr double kArrivedLast = 0.15;      // m, the last leg: the robot must
                                           // stand where the place's view is
constexpr double kNearMemory = 2.5;        // s to keep believing in something
                                           // seen close and then lost
constexpr double kNearRange = 0.45;        // m, what counts as close
constexpr double kNearSpeed = 0.07;        // m/s while it is remembered
constexpr double kStallSeconds = 4.0;      // no movement for this long is a stall
constexpr double kTurnInPlace = 0.9;       // rad; beyond this, turn before driving
}  // namespace

class DriveAction : public plansys2::ActionExecutorClient
{
public:
  DriveAction()
  : plansys2::ActionExecutorClient("drive_action")
  {
    declare_parameter<std::vector<std::string>>("roadmap_names", std::vector<std::string>{});
    declare_parameter<std::vector<double>>("roadmap_x", std::vector<double>{});
    declare_parameter<std::vector<double>>("roadmap_y", std::vector<double>{});
    // route_x and route_y: the places to drive through, in order, for each bay.
    declare_parameter<std::vector<std::string>>("route_x", std::vector<std::string>{});
    declare_parameter<std::vector<std::string>>("route_y", std::vector<std::string>{});
    declare_parameter<std::string>("robot", "r1");
    declare_parameter<std::string>("mode", "epistemic");
    // The event reported on arrival, with {b} for the bay: e-gate-{b}.
    declare_parameter<std::string>("outcome", "");
    // How the bay is chosen when the action does not name it: "order", the
    // bay the robot knows its order names; "gate", the gate it stands in.
    declare_parameter<std::string>("bay_from", "order");
    declare_parameter<std::vector<double>>("gate_x", std::vector<double>{});
    declare_parameter<std::vector<double>>("gate_y", std::vector<double>{});

    const std::string robot = get_parameter("robot").as_string();
    if (get_parameter("mode").as_string() == "epistemic") {
      state_ = std::make_shared<plansys2::EpistemicStateClient>(
        get_parameter("action_name").as_string() + "_" + robot + "_state_client");
    }
    odom_ = create_subscription<nav_msgs::msg::Odometry>(
      "/" + robot + "/odom", 10, [this](const nav_msgs::msg::Odometry::SharedPtr m) {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        px_ = m->pose.pose.position.x;
        py_ = m->pose.pose.position.y;
        const auto & q = m->pose.pose.orientation;
        pyaw_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
        have_pose_ = true;
      });

    cmd_ = create_publisher<geometry_msgs::msg::Twist>("/" + robot + "/cmd_vel", 10);
    scan_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/" + robot + "/scan", rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {on_scan(msg);});

  }

private:
  void on_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    // Every return within range, in the base frame. The window needs the
    // points themselves, not a single nearest distance: an obstacle to one
    // side rules out the arcs that curve into it and leaves the rest.
    std::vector<std::pair<double, double>> points;
    points.reserve(msg->ranges.size());
    for (std::size_t i = 0; i < msg->ranges.size(); ++i) {
      const double r = msg->ranges[i];
      if (!std::isfinite(r) || r < msg->range_min ||
          r > std::min(static_cast<double>(msg->range_max), 3.0))
      {
        continue;
      }
      const double a = msg->angle_min + i * msg->angle_increment;
      points.emplace_back(r * std::cos(a), r * std::sin(a));
    }
    // Anything close, remembered for a moment after it disappears. The laser
    // reports nothing inside 0.12 m, so an obstacle the robot is about to
    // touch is exactly the one it stops being able to see: the window then
    // finds clear space where the crate still is. Believing the last sighting
    // for a couple of seconds is what keeps the base off it.
    double nearest = std::numeric_limits<double>::infinity();
    for (const auto & p : points) {
      if (p.first > -0.10) {
        nearest = std::min(nearest, std::hypot(p.first, p.second));
      }
    }

    std::lock_guard<std::mutex> lock(scan_mutex_);
    obstacles_ = std::move(points);
    if (nearest < kNearRange) {
      near_until_ = now().seconds() + kNearMemory;
    }
  }

  /// Roll one (v, w) forward as a constant-curvature arc and report the
  /// closest any point of it comes to a laser return.
  double clearance_of(
    double v, double w, const std::vector<std::pair<double, double>> & obstacles) const
  {
    const double dt = kHorizon / kSimSteps;
    double x = 0.0, y = 0.0, th = 0.0;
    double closest = std::numeric_limits<double>::infinity();
    for (int k = 0; k < kSimSteps; ++k) {
      th += w * dt;
      x += v * std::cos(th) * dt;
      y += v * std::sin(th) * dt;
      for (const auto & o : obstacles) {
        const double d = std::hypot(o.first - x, o.second - y);
        if (d < closest) {closest = d;}
      }
    }
    return closest;
  }

  /// Endpoint of the same arc, for scoring progress towards the local goal.
  void arc_end(double v, double w, double & x, double & y) const
  {
    const double dt = kHorizon / kSimSteps;
    double th = 0.0;
    x = 0.0; y = 0.0;
    for (int k = 0; k < kSimSteps; ++k) {
      th += w * dt;
      x += v * std::cos(th) * dt;
      y += v * std::sin(th) * dt;
    }
  }

  /// The dynamic window: the best (v, w) for a local goal in the base frame,
  /// or nothing when every arc is inadmissible.
  bool choose(double gx, double gy, double & best_v, double & best_w)
  {
    std::vector<std::pair<double, double>> obstacles;
    {
      std::lock_guard<std::mutex> lock(scan_mutex_);
      obstacles = obstacles_;
    }

    const double v_lo = std::max(0.0, v_ - kAccelV * kControlDt);
    const double v_hi = std::min(kVMax, v_ + kAccelV * kControlDt);
    const double w_lo = std::max(-kWMax, w_ - kAccelW * kControlDt);
    const double w_hi = std::min(kWMax, w_ + kAccelW * kControlDt);

    const double here = std::hypot(gx, gy);
    double best_score = -std::numeric_limits<double>::infinity();
    bool found = false;

    for (int i = 0; i < kNv; ++i) {
      const double v = kNv == 1 ? v_lo : v_lo + (v_hi - v_lo) * i / (kNv - 1);
      for (int j = 0; j < kNw; ++j) {
        const double w = kNw == 1 ? w_lo : w_lo + (w_hi - w_lo) * j / (kNw - 1);

        const double clear = obstacles.empty() ?
          std::numeric_limits<double>::infinity() : clearance_of(v, w, obstacles);

        // Standing still collides with nothing; without this the base can
        // lock into recovery whenever a single return is closer than the
        // collision radius.
        const bool stationary = v < 1e-6;
        if (!stationary) {
          if (clear <= kCollision) {continue;}
          // Never carry more speed into a gap than can be shed before it.
          if (v > std::sqrt(2.0 * kAccelV * std::max(0.0, clear - kCollision))) {continue;}
        }

        double ex = 0.0, ey = 0.0;
        arc_end(v, w, ex, ey);
        const double to_goal = std::hypot(gx - ex, gy - ey);
        const double progress = (here - to_goal) / std::max(here, 1e-3);
        const double room = std::min(clear, kClearCap) / kClearCap;
        const double speed = v / kVMax;

        double score = kWGoal * progress + kWClear * room + kWSpeed * speed;
        if (stationary) {score -= 0.25;}   // never prefer sitting still

        if (score > best_score) {
          best_score = score;
          best_v = v;
          best_w = w;
          found = true;
        }
      }
    }
    return found;
  }

  /// No admissible arc: turn towards the side with more room, backing out if
  /// the nose is genuinely in something.
  void recover(double & v, double & w)
  {
    std::vector<std::pair<double, double>> obstacles;
    {
      std::lock_guard<std::mutex> lock(scan_mutex_);
      obstacles = obstacles_;
    }
    double front = std::numeric_limits<double>::infinity();
    double left = std::numeric_limits<double>::infinity();
    double right = std::numeric_limits<double>::infinity();
    for (const auto & o : obstacles) {
      const double d = std::hypot(o.first, o.second);
      if (o.first > -0.05 && std::fabs(o.second) < 0.4) {front = std::min(front, d);}
      if (o.second > 0.0) {left = std::min(left, d);} else {right = std::min(right, d);}
    }
    v = front < kCollision + 0.05 ? -0.09 : 0.0;
    w = (left > right) ? 1.2 : -1.2;
    ++recoveries_;
  }

  // Which bay the action concerns: named, known, or stood at.
  std::string bay(double rx, double ry)
  {
    const auto & args = get_arguments();
    if (!args.empty() && (args.back() == "bay_x" || args.back() == "bay_y")) {
      return args.back() == "bay_x" ? "x" : "y";
    }
    if (get_parameter("bay_from").as_string() == "gate") {
      const auto gx = get_parameter("gate_x").as_double_array();
      const auto gy = get_parameter("gate_y").as_double_array();
      if (gx.size() == 2 && gy.size() == 2) {
        return std::hypot(rx - gx[0], ry - gx[1]) < std::hypot(rx - gy[0], ry - gy[1]) ? "x" : "y";
      }
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

  bool route(std::vector<std::pair<double, double>> & legs, double rx, double ry)
  {
    bay_ = bay(rx, ry);
    if (bay_.empty()) {return false;}
    const auto names = get_parameter("roadmap_names").as_string_array();
    const auto xs = get_parameter("roadmap_x").as_double_array();
    const auto ys = get_parameter("roadmap_y").as_double_array();
    std::map<std::string, std::size_t> index;
    for (std::size_t i = 0; i < names.size() && i < xs.size() && i < ys.size(); ++i) {
      index[names[i]] = i;
    }
    const std::vector<std::string> places = get_parameter("route_" + bay_).as_string_array();
    legs.clear();
    for (const auto & p : places) {
      if (!index.count(p)) {return false;}
      legs.emplace_back(xs[index[p]], ys[index[p]]);
    }
    return !legs.empty();
  }

  bool pose(double & x, double & y, double & yaw)
  {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    if (!have_pose_) {return false;}
    x = px_;
    y = py_;
    yaw = pyaw_;
    return true;
  }

  void do_work() override
  {
    double x = 0.0, y = 0.0, yaw = 0.0;
    if (!pose(x, y, yaw)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000, "no odometry yet");
      send_feedback(0.0, "waiting for a pose");
      return;
    }
    // The route is fixed when the action starts and kept until it ends.
    if (!announced_ && !route(legs_, x, y)) {
      std::string args;
      for (const auto & a : get_arguments()) {args += a + " ";}
      RCLCPP_ERROR(
        get_logger(), "no waypoint for arguments [%s]", args.c_str());
      finish(false, 0.0, "the action names no route this node knows");
      return;
    }
    const auto & legs = legs_;
    if (leg_ >= legs.size()) {leg_ = 0;}
    double tx = legs[leg_].first;
    double ty = legs[leg_].second;
    if (!announced_) {
      announced_ = true;
      std::string args;
      for (const auto & a : get_arguments()) {args += a + " ";}
      RCLCPP_INFO(
        get_logger(), "driving [%s] over %zu legs", args.c_str(), legs.size());
    }

    const double dx = tx - x, dy = ty - y;
    const double range = std::hypot(dx, dy);
    if (range < (leg_ + 1 < legs.size() ? kArrived : kArrivedLast)) {
      if (leg_ + 1 < legs.size()) {
        // A leg reached is not the action done: pick up the next one.
        ++leg_;
        RCLCPP_INFO(
          get_logger(), "leg %zu/%zu reached", leg_, legs.size());
        start_range_ = -1.0;
        send_feedback(
          static_cast<float>(leg_) / static_cast<float>(legs.size()), "driving");
        return;
      }
      cmd_->publish(geometry_msgs::msg::Twist());
      std::string outcome = get_parameter("outcome").as_string();
      const auto at = outcome.find("{b}");
      if (at != std::string::npos) {outcome.replace(at, 3, bay_);}
      RCLCPP_INFO(
        get_logger(), "arrived at (%.2f, %.2f), bay %s%s%s", tx, ty, bay_.c_str(),
        outcome.empty() ? "" : "; event ", outcome.c_str());
      announced_ = false;
      leg_ = 0;
      start_range_ = -1.0;
      stall_since_ = -1.0;
      finish(true, 1.0, "arrived", outcome);
      return;
    }

    // The goal, in the frame the window is sampled in.
    const double c = std::cos(-yaw), sn = std::sin(-yaw);
    const double gx = c * dx - sn * dy;
    const double gy = sn * dx + c * dy;

    // A stall watchdog. The window can be perfectly reasonable and the robot
    // still not move: nose against something the laser cannot see because it
    // is inside the 0.12 m minimum range, or an arc that is admissible but
    // makes no progress. Left alone that is a run that never ends, so a base
    // that has not moved for kStallSeconds is backed out and turned before it
    // is asked to drive again.
    const double t = now().seconds();
    if (std::hypot(x - stall_x_, y - stall_y_) > 0.06) {
      stall_x_ = x;
      stall_y_ = y;
      stall_since_ = t;
    }
    if (stall_since_ > 0.0 && t - stall_since_ > kStallSeconds) {
      geometry_msgs::msg::Twist out;
      double v = 0.0, w = 0.0;
      recover(v, w);
      out.linear.x = v < 0.0 ? v : -0.10;
      out.angular.z = w;
      cmd_->publish(out);
      v_ = out.linear.x;
      w_ = out.angular.z;
      if (t - stall_since_ > kStallSeconds + 2.0) {
        stall_since_ = t;   // one backing-out at a time, then try again
        RCLCPP_WARN(get_logger(), "stalled at (%.2f, %.2f); backing out", x, y);
      }
      send_feedback(0.0f, "unsticking");
      return;
    }
    if (stall_since_ <= 0.0) {
      stall_since_ = t;
      stall_x_ = x;
      stall_y_ = y;
    }

    geometry_msgs::msg::Twist out;
    const double bearing = std::atan2(gy, gx);

    double v = 0.0, w = 0.0;
    if (std::fabs(bearing) > kTurnInPlace) {
      // Facing away from the goal: no forward arc scores well, and swinging
      // wide through whatever is beside us is worse than turning on the spot.
      v = 0.0;
      w = std::copysign(1.2, bearing);
    } else if (!choose(gx, gy, v, w)) {
      recover(v, w);
    }

    // While something close is still remembered, creep rather than drive.
    {
      std::lock_guard<std::mutex> lock(scan_mutex_);
      if (now().seconds() < near_until_) {
        v = std::min(v, kNearSpeed);
      }
    }

    out.linear.x = v;
    out.angular.z = std::clamp(w, -kWMax, kWMax);
    cmd_->publish(out);

    // The window is centred on what was last commanded, not on odometry: a
    // window centred on a measurement that lags closes on itself, and nothing
    // faster than one acceleration step is ever sampled.
    v_ = out.linear.x;
    w_ = out.angular.z;

    if (start_range_ < 0.0) {start_range_ = range;}
    const double within = std::clamp(1.0 - range / std::max(start_range_, 1e-3), 0.0, 1.0);
    send_feedback(
      static_cast<float>((leg_ + within) / static_cast<double>(legs.size())), "driving");
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_;
  std::shared_ptr<plansys2::EpistemicStateClient> state_;
  std::mutex pose_mutex_;
  bool have_pose_{false};
  double px_{0.0}, py_{0.0}, pyaw_{0.0};
  std::string bay_;
  std::vector<std::pair<double, double>> obstacles_;
  std::mutex scan_mutex_;
  double v_{0.0};
  double w_{0.0};
  std::size_t recoveries_{0};
  double stall_x_{0.0};
  double stall_y_{0.0};
  double stall_since_{-1.0};
  double near_until_{0.0};
  bool announced_{false};
  std::size_t leg_{0};
  double start_range_{-1.0};
  std::vector<std::pair<double, double>> legs_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DriveAction>();
  // action_name and robot come from the launch file.
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
