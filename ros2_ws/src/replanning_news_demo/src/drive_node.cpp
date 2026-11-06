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

// The drives of replanning on news: to a door's mouth or the kitchen hatch
// (goto), through a door into the north corridor (cross), and along the
// corridor to the resident (deliver). The controller is collapsed_wing_demo's,
// the dynamic window of gate_signal_demo's drive node, on the robot's own
// odometry. What is new is the route.
//
// A drive may begin anywhere: a policy interrupted by news leaves the robot
// part of the way along a drive, and the drive the new policy asks for starts
// from there. So a goto or a deliver is routed over its graph (the south half
// as far as each door, or the north corridor), from whichever end of the
// graph's nearest edge makes the route shortest, to its target. A crossing
// keeps to its own passage, on route_fixed.
//
// When the executor cancels a drive the base is stopped at once: a robot whose
// policy was abandoned must not coast on its last command. With
// stop_when_blocked, a drive that finds returns of its own laser on its route
// ahead for kBlockedSeconds stops and FAILS with "shut": the classical courier
// finding a door it assumed open.

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
#include <queue>
#include <functional>

#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "rclcpp/rclcpp.hpp"

#include "lifecycle_msgs/msg/transition.hpp"

#include "nav_msgs/msg/odometry.hpp"
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
constexpr double kArrivedLast = 0.15;      // m, the last leg: the robot must
                                           // stand where the place's view is
constexpr double kNearMemory = 2.5;        // s to keep believing in something
                                           // seen close and then lost
constexpr double kNearRange = 0.45;        // m, what counts as close
constexpr double kNearSpeed = 0.07;        // m/s while it is remembered
constexpr double kStallSeconds = 4.0;      // no movement for this long is a stall
constexpr double kTurnInPlace = 0.9;       // rad; beyond this, turn before driving
constexpr double kOnRoute = 0.25;          // m from the route: a return on it
constexpr double kAhead = 1.0;             // m along the route, looked at for rubble
constexpr double kBlockedSeconds = 2.0;    // returns on the route for this long: blocked
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
    // The graph a goto or a deliver is routed over, as "u v" pairs.
    declare_parameter<std::vector<std::string>>("edges", std::vector<std::string>{});
    declare_parameter<std::string>("target", "");
    declare_parameter<std::vector<std::string>>("route_fixed", std::vector<std::string>{});
    declare_parameter<std::string>("robot", "c");
    declare_parameter<bool>("stop_when_blocked", false);

    const std::string robot = get_parameter("robot").as_string();
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

  plansys2::CallbackReturnT on_deactivate(const rclcpp_lifecycle::State & state) override
  {
    // Cancelled, or finished: either way the base stops here.
    cmd_->publish(geometry_msgs::msg::Twist());
    v_ = w_ = 0.0;
    if (announced_) {
      double x = 0.0, y = 0.0, yaw = 0.0;
      pose(x, y, yaw);
      RCLCPP_WARN(get_logger(), "stopped at (%.2f, %.2f) before the drive ended", x, y);
    }
    reset();
    return plansys2::ActionExecutorClient::on_deactivate(state);
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

  void reset()
  {
    announced_ = false;
    leg_ = 0;
    start_range_ = -1.0;
    stall_since_ = -1.0;
    blocked_since_ = -1.0;
  }

  /// The places to drive through, in order, from (rx, ry).
  bool route(std::vector<std::pair<double, double>> & legs, double rx, double ry)
  {
    const auto names = get_parameter("roadmap_names").as_string_array();
    const auto xs = get_parameter("roadmap_x").as_double_array();
    const auto ys = get_parameter("roadmap_y").as_double_array();
    std::map<std::string, std::size_t> index;
    for (std::size_t i = 0; i < names.size() && i < xs.size() && i < ys.size(); ++i) {
      index[names[i]] = i;
    }
    std::vector<std::string> places = get_parameter("route_fixed").as_string_array();
    if (places.empty()) {
      const std::string target = get_parameter("target").as_string();
      if (!index.count(target)) {return false;}
      // The graph, and the shortest distance from every node to the target.
      std::map<std::string, std::vector<std::pair<std::string, double>>> adj;
      std::vector<std::pair<std::string, std::string>> edges;
      // A copy: the array belongs to the Parameter get_parameter returns, which
      // dies at the end of the full expression.
      const std::vector<std::string> edge_list = get_parameter("edges").as_string_array();
      for (const auto & e : edge_list) {
        std::istringstream in(e);
        std::string u, v;
        in >> u >> v;
        if (!index.count(u) || !index.count(v)) {continue;}
        const double w = std::hypot(xs[index[u]] - xs[index[v]], ys[index[u]] - ys[index[v]]);
        adj[u].emplace_back(v, w);
        adj[v].emplace_back(u, w);
        edges.emplace_back(u, v);
      }
      std::map<std::string, double> dist;
      std::map<std::string, std::string> next;
      using Item = std::pair<double, std::string>;
      std::priority_queue<Item, std::vector<Item>, std::greater<Item>> todo;
      dist[target] = 0.0;
      todo.emplace(0.0, target);
      while (!todo.empty()) {
        const auto [d, u] = todo.top();
        todo.pop();
        if (d > dist[u]) {continue;}
        for (const auto & [v, w] : adj[u]) {
          if (!dist.count(v) || d + w < dist[v]) {
            dist[v] = d + w;
            next[v] = u;
            todo.emplace(dist[v], v);
          }
        }
      }
      // The nearest edge, and the end of it that makes the route shortest.
      double best_d = std::numeric_limits<double>::infinity();
      std::pair<std::string, std::string> nearest;
      for (const auto & [u, v] : edges) {
        const double ax = xs[index[u]], ay = ys[index[u]], bx = xs[index[v]], by = ys[index[v]];
        const double lx = bx - ax, ly = by - ay;
        const double t = std::clamp(((rx - ax) * lx + (ry - ay) * ly) / std::max(lx * lx + ly * ly, 1e-9), 0.0, 1.0);
        const double d = std::hypot(ax + t * lx - rx, ay + t * ly - ry);
        if (d < best_d) {best_d = d; nearest = {u, v};}
      }
      std::string first;
      double best = std::numeric_limits<double>::infinity();
      for (const auto & u : {nearest.first, nearest.second}) {
        if (!dist.count(u)) {continue;}
        const double c = std::hypot(xs[index[u]] - rx, ys[index[u]] - ry) + dist[u];
        if (c < best) {best = c; first = u;}
      }
      if (first.empty()) {return false;}
      places.clear();
      for (std::string u = first; ; u = next[u]) {
        places.push_back(u);
        if (u == target) {break;}
      }
      // A first place behind the robot, on the way to the second, is skipped.
      if (places.size() > 1) {
        const auto & a = places[0];
        const auto & b = places[1];
        const double ab = std::hypot(xs[index[b]] - xs[index[a]], ys[index[b]] - ys[index[a]]);
        const double rb = std::hypot(xs[index[b]] - rx, ys[index[b]] - ry);
        if (rb < ab) {places.erase(places.begin());}
      }
    }
    places_ = places;
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
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "no odometry yet");
      send_feedback(0.0, "waiting for a pose");
      return;
    }
    // The route is fixed when the action starts and kept until it ends.
    if (!announced_) {
      if (!route(legs_, x, y)) {
        RCLCPP_ERROR(get_logger(), "no route to %s", get_parameter("target").as_string().c_str());
        finish(false, 0.0, "no route");
        return;
      }
      announced_ = true;
      leg_ = 0;
      std::string via;
      for (const auto & p : places_) {via += p + " ";}
      RCLCPP_INFO(get_logger(), "driving from (%.2f, %.2f) by %s", x, y, via.c_str());
    }
    const auto & legs = legs_;
    if (leg_ >= legs.size()) {leg_ = 0;}
    const double tx = legs[leg_].first;
    const double ty = legs[leg_].second;
    const double dx = tx - x, dy = ty - y;
    const double range = std::hypot(dx, dy);
    if (range < (leg_ + 1 < legs.size() ? kArrived : kArrivedLast)) {
      if (leg_ + 1 < legs.size()) {
        ++leg_;
        start_range_ = -1.0;
        send_feedback(static_cast<float>(leg_) / static_cast<float>(legs.size()), "driving");
        return;
      }
      cmd_->publish(geometry_msgs::msg::Twist());
      v_ = w_ = 0.0;
      RCLCPP_INFO(get_logger(), "arrived at (%.2f, %.2f)", tx, ty);
      reset();
      finish(true, 1.0, "arrived");
      return;
    }

    // The goal, in the frame the window is sampled in.
    const double c = std::cos(-yaw), sn = std::sin(-yaw);
    const double gx = c * dx - sn * dy;
    const double gy = sn * dx + c * dy;

    // A shut door on the route: returns of the robot's own laser within kOnRoute of
    // the route over the next kAhead of it, held for kBlockedSeconds.
    if (get_parameter("stop_when_blocked").as_bool()) {
      std::vector<std::pair<double, double>> route_ahead;
      double px = x, py = y, left = kAhead;
      for (std::size_t k = leg_; k < legs.size() && left > 0.0; ++k) {
        const double lx = legs[k].first - px, ly = legs[k].second - py;
        const double len = std::hypot(lx, ly);
        for (double s = 0.0; s < std::min(len, left); s += 0.05) {
          route_ahead.emplace_back(px + lx * s / std::max(len, 1e-9), py + ly * s / std::max(len, 1e-9));
        }
        left -= len;
        px = legs[k].first;
        py = legs[k].second;
      }
      std::vector<std::pair<double, double>> points;
      {
        std::lock_guard<std::mutex> lock(scan_mutex_);
        points = obstacles_;
      }
      const double cy_ = std::cos(yaw), sy_ = std::sin(yaw);
      bool on_route = false;
      for (const auto & o : points) {
        const double wx = x + cy_ * o.first - sy_ * o.second;
        const double wy = y + sy_ * o.first + cy_ * o.second;
        for (const auto & r : route_ahead) {
          if (std::hypot(wx - r.first, wy - r.second) < kOnRoute) {on_route = true; break;}
        }
        if (on_route) {break;}
      }
      const double tb = now().seconds();
      if (!on_route) {
        blocked_since_ = -1.0;
      } else if (blocked_since_ < 0.0) {
        blocked_since_ = tb;
      } else if (tb - blocked_since_ > kBlockedSeconds) {
        cmd_->publish(geometry_msgs::msg::Twist());
        v_ = w_ = 0.0;
        RCLCPP_WARN(get_logger(), "the door is shut ahead, at (%.2f, %.2f)", x, y);
        reset();
        finish(false, 1.0, "shut");
        return;
      }
    }

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
  std::mutex pose_mutex_;
  bool have_pose_{false};
  double px_{0.0}, py_{0.0}, pyaw_{0.0};
  std::vector<std::pair<double, double>> obstacles_;
  std::mutex scan_mutex_;
  double v_{0.0};
  double w_{0.0};
  std::size_t recoveries_{0};
  double stall_x_{0.0};
  double stall_y_{0.0};
  double stall_since_{-1.0};
  double blocked_since_{-1.0};
  double near_until_{0.0};
  bool announced_{false};
  std::size_t leg_{0};
  double start_range_{-1.0};
  std::vector<std::pair<double, double>> legs_;
  std::vector<std::string> places_;
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
