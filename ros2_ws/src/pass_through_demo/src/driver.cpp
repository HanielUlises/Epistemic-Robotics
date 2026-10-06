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

#include "pass_through_demo/driver.hpp"

#include <algorithm>
#include <cmath>

#include <nlohmann/json.hpp>

namespace pass_through
{

namespace
{

double wrap(double a)
{
  while (a > M_PI) {a -= 2.0 * M_PI;}
  while (a < -M_PI) {a += 2.0 * M_PI;}
  return a;
}

}  // namespace

Driver::Driver(rclcpp::Node::SharedPtr node, const Config & config)
: node_(std::move(node)), config_(config), floorplan_(load_map_yaml(config.floorplan))
{
  // Stamped from the node's own clock. A default-constructed rclcpp::Time is
  // on the system clock and now() is on ROS time, and subtracting the two
  // throws -- on the first control step, from inside the performer's timer.
  planned_at_ = node_->now();
  blocked_since_ = rclcpp::Time(0, 0, planned_at_.get_clock_type());
  backing_until_ = rclcpp::Time(0, 0, planned_at_.get_clock_type());

  const std::string ns = "/" + config_.ns + "/";
  const auto latched = rclcpp::QoS(1).transient_local().reliable();

  cmd_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(ns + "cmd_vel", 10);
  route_pub_ = node_->create_publisher<nav_msgs::msg::Path>(ns + "route", latched);
  region_pub_ = node_->create_publisher<nav_msgs::msg::OccupancyGrid>(ns + "winning_region", latched);

  odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
    ns + "odom", rclcpp::SensorDataQoS(),
    [this](nav_msgs::msg::Odometry::SharedPtr m) {on_odom(m);});
  scan_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
    ns + "scan", rclcpp::SensorDataQoS(),
    [this](sensor_msgs::msg::LaserScan::SharedPtr m) {on_scan(m);});
  known_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
    ns + "known_map", latched,
    [this](nav_msgs::msg::OccupancyGrid::SharedPtr m) {on_known(m);});
  // The model, as the epistemic state publishes it after every update. Read
  // here rather than asked for with check_formula, because the formula the
  // safe set is built from is evaluated over the whole grid and not once.
  state_sub_ = node_->create_subscription<std_msgs::msg::String>(
    "/epistemic_state/state", rclcpp::QoS(10).transient_local(),
    [this](std_msgs::msg::String::SharedPtr m) {on_state(m);});
}

void Driver::on_odom(nav_msgs::msg::Odometry::SharedPtr msg)
{
  // The Waffle's odometry is in the world frame -- Gazebo's diff drive reports
  // the model's world pose -- and so is every map here. No transform is
  // looked up because none is needed; see warehouse_demo's note on the map
  // frame for what goes wrong when that stops being true.
  std::lock_guard<std::mutex> held(lock_);
  x_ = msg->pose.pose.position.x;
  y_ = msg->pose.pose.position.y;
  const auto & q = msg->pose.pose.orientation;
  yaw_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  have_pose_ = true;
}

void Driver::on_scan(sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  std::lock_guard<std::mutex> held(lock_);
  scan_ = std::move(msg);
}

void Driver::on_known(nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  std::lock_guard<std::mutex> held(lock_);
  known_ = from_msg(*msg);
  ++knowledge_version_;
}

void Driver::on_state(std_msgs::msg::String::SharedPtr msg)
{
  try {
    const auto payload = nlohmann::json::parse(msg->data);
    if (!payload.contains("model")) {
      return;
    }
    const auto model = payload["model"].dump();
    std::lock_guard<std::mutex> held(lock_);
    if (model != model_json_) {
      model_json_ = model;
      ++knowledge_version_;
    }
  } catch (const std::exception &) {
  }
}

bool Driver::pose(double & x, double & y, double & yaw) const
{
  std::lock_guard<std::mutex> held(lock_);
  x = x_;
  y = y_;
  yaw = yaw_;
  return have_pose_;
}

bool Driver::blocked_ahead() const
{
  std::lock_guard<std::mutex> held(lock_);
  if (!scan_) {
    return false;
  }
  // The forward sector, forty degrees either side. The laser sits 6 cm behind
  // the base's centre; the margin in `veto` absorbs that.
  const double half = 40.0 * M_PI / 180.0;
  for (std::size_t i = 0; i < scan_->ranges.size(); ++i) {
    const double a = wrap(scan_->angle_min + i * scan_->angle_increment);
    const float r = scan_->ranges[i];
    if (std::fabs(a) > half || !std::isfinite(r) || r < scan_->range_min) {
      continue;
    }
    // Wider at the sides of the sector than straight ahead: what matters is
    // what the chassis would sweep, not the distance along the ray.
    if (r * std::cos(a) < config_.veto && std::fabs(r * std::sin(a)) < 0.22) {
      return true;
    }
  }
  return false;
}

Driver::Assessment Driver::assess(double gx, double gy, double radius, bool publish)
{
  Assessment out;
  Grid known;
  std::string model;
  double x, y, yaw;
  {
    std::lock_guard<std::mutex> held(lock_);
    known = known_;
    model = model_json_;
    x = x_;
    y = y_;
    yaw = yaw_;
  }
  (void)yaw;

  SafeSetInput input;
  input.floorplan = &floorplan_;
  input.knowledge = known.empty() ? nullptr : &known;
  input.bays = config_.bays;
  input.agent = config_.agent;
  input.model_json = model;
  input.inflation = config_.inflation;
  const auto safe = build_safe_set(input);
  out.formula = safe.formula;
  if (!safe.ok) {
    out.error = safe.error;
    return out;
  }
  out.lifted = safe.lifted;
  out.known_open = safe.known_open;

  std::vector<char> goal(safe.safe.size(), 0);
  for (const auto i : floorplan_.cells_in(Box{gx - radius, gy - radius, gx + radius, gy + radius})) {
    double cx, cy;
    floorplan_.centre(i, cx, cy);
    if (std::hypot(cx - gx, cy - gy) <= radius && safe.safe[i]) {
      goal[i] = 1;
    }
  }
  const auto reach = least_fixed_point(safe.graph, goal, safe.safe);
  out.ok = true;
  out.region = reach.size;
  out.iterations = reach.iterations;
  if (publish) {
    publish_region(reach, safe);
  }

  std::size_t here = 0, start = 0;
  if (!floorplan_.index(x, y, here)) {
    out.error = "the robot is off the grid";
    return out;
  }
  // A robot parked beside a shelf stands in the inflation band, which is not
  // in the safe set; the route starts from the nearest cell that is, within
  // half a metre.
  std::vector<char> in_region(reach.layer.size(), 0);
  for (std::size_t i = 0; i < in_region.size(); ++i) {
    in_region[i] = reach.contains(i) ? 1 : 0;
  }
  if (!nearest_in(floorplan_, in_region, here, 5, start)) {
    // No route, so none is drawn: a latched route left over from an earlier
    // answer would show a way the agent no longer has.
    if (publish) {
      publish_route({});
    }
    return out;
  }
  out.reachable = true;

  // Shortcuts are taken against the safe set less one cell, so a straight
  // segment runs no closer to an obstacle than the inflation and a cell.
  const auto path = shortcut(floorplan_, erode(floorplan_, safe.safe),
      descend(safe.graph, reach, start));
  std::vector<std::pair<double, double>> route;
  route.emplace_back(x, y);
  for (const auto i : path) {
    double cx, cy;
    floorplan_.centre(i, cx, cy);
    route.emplace_back(cx, cy);
  }
  for (std::size_t k = 1; k < route.size(); ++k) {
    out.route_length += std::hypot(
      route[k].first - route[k - 1].first, route[k].second - route[k - 1].second);
  }
  // Which bays the route passes through, sampled along it.
  for (const auto & [name, box] : config_.bays) {
    bool crosses = false;
    for (std::size_t k = 1; k < route.size() && !crosses; ++k) {
      for (int s = 0; s <= 20 && !crosses; ++s) {
        const double t = s / 20.0;
        crosses = box.contains(
          route[k - 1].first + t * (route[k].first - route[k - 1].first),
          route[k - 1].second + t * (route[k].second - route[k - 1].second));
      }
    }
    if (crosses) {out.route_bays.push_back(name);}
  }

  if (publish) {
    std::lock_guard<std::mutex> held(lock_);
    route_ = route;
    route_at_ = 1;
    publish_route(path);
  }
  return out;
}

Driver::Progress Driver::step_to(double gx, double gy, double radius)
{
  double x, y, yaw;
  if (!pose(x, y, yaw)) {
    return Progress::Waiting;
  }
  if (std::hypot(gx - x, gy - y) <= radius) {
    stop();
    return Progress::Arrived;
  }

  const bool new_goal = std::hypot(gx - goal_x_, gy - goal_y_) > 1e-3;
  std::uint64_t version;
  {
    std::lock_guard<std::mutex> held(lock_);
    version = knowledge_version_;
  }
  const bool stale = (node_->now() - planned_at_).seconds() > config_.replan_period;
  if (new_goal || route_.empty() || (stale && version != planned_version_)) {
    goal_x_ = gx;
    goal_y_ = gy;
    planned_at_ = node_->now();
  blocked_since_ = rclcpp::Time(0, 0, planned_at_.get_clock_type());
  backing_until_ = rclcpp::Time(0, 0, planned_at_.get_clock_type());
    planned_version_ = version;
    const auto a = assess(gx, gy, radius, true);
    if (!a.ok || !a.reachable) {
      std::lock_guard<std::mutex> held(lock_);
      route_.clear();
      stop();
      return Progress::NoRoute;
    }
  }

  std::vector<std::pair<double, double>> route;
  std::size_t at;
  {
    std::lock_guard<std::mutex> held(lock_);
    route = route_;
    at = route_at_;
  }
  if (route.size() < 2) {
    stop();
    return Progress::NoRoute;
  }

  // Pure pursuit: the first point of the route at least a lookahead away,
  // having dropped the points already passed.
  while (at + 1 < route.size() &&
    std::hypot(route[at].first - x, route[at].second - y) < config_.lookahead)
  {
    ++at;
  }
  {
    std::lock_guard<std::mutex> held(lock_);
    route_at_ = at;
  }
  const auto [tx, ty] = route[at];
  const double error = wrap(std::atan2(ty - y, tx - x) - yaw);

  // Recovery. The laser has held the robot for two seconds: something the
  // floor plan does not have is in front of it, or the follower has cut a
  // corner into the inflation band. Back off a little and plan again from
  // where it then is; standing still with the obstacle in front of it is not
  // going to change anything.
  const auto clock = node_->now();
  if (backing_until_.nanoseconds() > 0) {
    if (clock < backing_until_) {
      geometry_msgs::msg::Twist back;
      back.linear.x = -0.15;
      cmd_pub_->publish(back);
      return Progress::Moving;
    }
    backing_until_ = rclcpp::Time(0, 0, clock.get_clock_type());
    route_.clear();
    return Progress::Moving;
  }

  geometry_msgs::msg::Twist cmd;
  if (std::fabs(error) > 0.7) {
    cmd.angular.z = std::copysign(std::min(config_.turn_rate, 2.0 * std::fabs(error)), error);
  } else {
    const double remaining = std::hypot(gx - x, gy - y);
    cmd.linear.x = config_.speed * std::cos(error) * std::min(1.0, 0.5 + remaining / 2.0);
    cmd.angular.z = std::clamp(2.2 * error, -config_.turn_rate, config_.turn_rate);
    if (blocked_ahead()) {
      cmd.linear.x = 0.0;
      if (blocked_since_.nanoseconds() == 0) {
        blocked_since_ = clock;
      } else if ((clock - blocked_since_).seconds() > 2.0) {
        RCLCPP_WARN(
          node_->get_logger(), "[drive] %s held by its laser at (%.2f, %.2f) for 2 s; backing off "
          "and replanning", config_.agent.c_str(), x, y);
        blocked_since_ = rclcpp::Time(0, 0, clock.get_clock_type());
        backing_until_ = clock + rclcpp::Duration::from_seconds(1.5);
      }
    } else {
      blocked_since_ = rclcpp::Time(0, 0, clock.get_clock_type());
    }
  }
  cmd_pub_->publish(cmd);
  return Progress::Moving;
}

bool Driver::face(double target, double tolerance)
{
  double x, y, yaw;
  if (!pose(x, y, yaw)) {
    return false;
  }
  const double error = wrap(target - yaw);
  if (std::fabs(error) <= tolerance) {
    stop();
    return true;
  }
  geometry_msgs::msg::Twist cmd;
  cmd.angular.z = std::copysign(std::clamp(1.5 * std::fabs(error), 0.25, config_.turn_rate), error);
  cmd_pub_->publish(cmd);
  return false;
}

void Driver::creep(double speed)
{
  geometry_msgs::msg::Twist cmd;
  // The laser looks forward, so it vetoes forward motion only. Backing up
  // retraces floor the robot has just crossed.
  cmd.linear.x = (speed > 0.0 && blocked_ahead()) ? 0.0 : speed;
  cmd_pub_->publish(cmd);
}

void Driver::stop()
{
  cmd_pub_->publish(geometry_msgs::msg::Twist{});
}

void Driver::publish_route(const std::vector<std::size_t> & cells)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  path.header.stamp = node_->now();
  for (const auto i : cells) {
    geometry_msgs::msg::PoseStamped p;
    p.header = path.header;
    floorplan_.centre(i, p.pose.position.x, p.pose.position.y);
    p.pose.position.z = 0.05;
    p.pose.orientation.w = 1.0;
    path.poses.push_back(p);
  }
  route_pub_->publish(path);
}

void Driver::publish_region(const Reach & reach, const SafeSet & safe)
{
  // The winning region, drawn as a costmap: 0 is outside and transparent in
  // that palette, the region is one flat value, and a bay the formula lifted
  // is drawn in the palette's one cyan, 99, so the cells that are safe only
  // because the agent knows something can be seen as such.
  Grid g = blank_like(floorplan_, 0);
  std::vector<char> lifted(g.size(), 0);
  for (const auto & name : safe.lifted) {
    for (const auto i : g.cells_in(config_.bays.at(name))) {
      lifted[i] = 1;
    }
  }
  for (std::size_t i = 0; i < g.size(); ++i) {
    if (reach.contains(i)) {
      g.cells[i] = lifted[i] ? 99 : 30;
    }
  }
  auto msg = to_msg(g, "map");
  msg.header.stamp = node_->now();
  region_pub_->publish(msg);
}

}  // namespace pass_through
