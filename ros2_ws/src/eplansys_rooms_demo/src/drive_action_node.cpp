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

// The ontic half of the demo: what `goto-door3` and `goto-door6` mean for a
// robot that has wheels.
//
// ePlanSys renders a policy as a behavior tree and drives each action through
// an ActionExecutorClient; what the action *does* is deliberately not its
// business, and no planning package can supply it. This node is that supply,
// and it is the only bespoke code in the demo.
//
// Driving is a straight-line pursuit of a named waypoint with a laser guard:
// the building's rooms are large and its doors are wide, the route between
// them is a short leg down the cross corridor, and a planner-grade local
// planner would be more machinery than the geometry needs. The guard is not
// optional, though -- the map is being built while the robot moves, and an
// obstacle it has not mapped is still an obstacle.

#include <algorithm>
#include <cmath>
#include <map>
#include <mutex>
#include <utility>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "lifecycle_msgs/msg/transition.hpp"

#include "plansys2_executor/ActionExecutorClient.hpp"

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
    // A route per destination, as flat [x1,y1,x2,y2,...] in the map frame.
    //
    // A route rather than a point, because the building has walls: a straight
    // line from room 1 to the east wing crosses three of them, and this node
    // has a laser guard rather than a planner -- it stops for what it sees, it
    // does not go around it. The legs below follow the corridors, and the
    // guard is what handles the furniture standing in them.
    //
    // Both east rooms open off the same narrow band around y = 0: the wall at
    // x = 4 runs from y = 0.8 up and from y = -0.8 down, so the doorways are
    // the gap between those two segments.
    declare_parameter<std::vector<double>>(
      "route_door3", {-6.0, 6.0, -6.0, 1.5, -2.0, 1.5, 1.0, 0.45, 3.0, 0.45, 4.9, 0.45, 5.7, 1.3});
    declare_parameter<std::vector<double>>(
      "route_door6", {4.9, 0.45, 3.0, 0.45, 3.0, -0.45, 4.9, -0.45, 6.2, -1.7});

    cmd_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    scan_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {on_scan(msg);});

    buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);
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

  bool route(std::vector<std::pair<double, double>> & legs)
  {
    const auto & args = get_arguments();
    if (args.empty()) {return false;}
    const std::string name = "route_" + args.back();
    if (!has_parameter(name)) {return false;}
    const auto flat = get_parameter(name).as_double_array();
    if (flat.size() < 2 || flat.size() % 2 != 0) {return false;}
    legs.clear();
    for (std::size_t i = 0; i + 1 < flat.size(); i += 2) {
      legs.emplace_back(flat[i], flat[i + 1]);
    }
    return true;
  }

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
    std::vector<std::pair<double, double>> legs;
    if (!route(legs)) {
      std::string args;
      for (const auto & a : get_arguments()) {args += a + " ";}
      RCLCPP_ERROR(
        get_logger(), "no waypoint for arguments [%s]", args.c_str());
      finish(false, 0.0, "the action names no route this node knows");
      return;
    }
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

    double x = 0.0, y = 0.0, yaw = 0.0;
    if (!pose(x, y, yaw)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000, "no map -> base_footprint transform yet");
      // SLAM has not published map -> odom yet. Waiting is right: the action
      // has not failed, it has not started.
      send_feedback(0.0, "waiting for a pose");
      return;
    }

    const double dx = tx - x, dy = ty - y;
    const double range = std::hypot(dx, dy);
    if (range < kArrived) {
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
      RCLCPP_INFO(get_logger(), "arrived at (%.2f, %.2f)", tx, ty);
      announced_ = false;
      leg_ = 0;
      start_range_ = -1.0;
      stall_since_ = -1.0;
      finish(true, 1.0, "arrived");
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
  std::shared_ptr<tf2_ros::Buffer> buffer_;
  std::shared_ptr<tf2_ros::TransformListener> listener_;
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
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DriveAction>();
  // action_name comes from the launch file: one instance per goto action.
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
