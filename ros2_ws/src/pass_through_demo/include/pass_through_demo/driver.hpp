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

#ifndef PASS_THROUGH_DEMO__DRIVER_HPP_
#define PASS_THROUGH_DEMO__DRIVER_HPP_

#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "pass_through_demo/grid.hpp"
#include "pass_through_demo/reach.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"

namespace pass_through
{

/// Drives one robot, on routes the least fixed point finds over what that
/// robot knows.
///
/// It plans nothing a performer did not ask for. Asked for a place, it builds
/// the agent's safe set from the floor plan, its knowledge map and the current
/// epistemic model, takes the least fixed point towards the place, descends it
/// from where the robot stands, and follows the result. When the robot is not
/// in the winning region there is no route, and it says so and does not move:
/// the fixed point is the authority on whether the agent may go, not the
/// controller.
///
/// It lives on a plain node of its own, beside the performer's lifecycle node,
/// so that its topics are up whether or not the performer is active.
class Driver
{
public:
  struct Config
  {
    std::string agent;             ///< EPDDL agent, the index of K
    std::string ns;                ///< robot namespace, r1 etc.
    std::string floorplan;         ///< map_server YAML
    std::map<std::string, Box> bays;
    double inflation{0.35};
    double speed{0.55};            ///< m/s on a straight
    double turn_rate{1.2};         ///< rad/s at most
    double lookahead{0.6};         ///< m
    double veto{0.32};             ///< m ahead within which the laser stops it
    double replan_period{2.0};     ///< s
    /// Namespaces of the other robots, whose odometry is read so that a
    /// route keeps off where they stand. Empty on the pass-through floor.
    std::vector<std::string> others;
  };

  enum class Progress {Moving, Arrived, NoRoute, Waiting};

  Driver(rclcpp::Node::SharedPtr node, const Config & config);

  /// One control step towards (x, y). Call it at the performer's rate.
  Progress step_to(double x, double y, double radius);

  /// Turn on the spot to `yaw`. True once facing it.
  bool face(double yaw, double tolerance = 0.08);

  /// Creep straight ahead at `speed`; the laser still vetoes.
  void creep(double speed);

  /// Creep at `speed` along the line through (x0, y0) at `heading`, steering
  /// back onto it; the laser still vetoes. In the simulator a base that has
  /// just turned on the spot drifts back towards its old heading when it is
  /// then driven straight, about forty degrees over a metre and a half, and
  /// an open-loop creep follows the drift.
  void creep(double speed, double x0, double y0, double heading);

  void stop();

  /// Boxes routes may not enter until set again; empty to lift them. A new
  /// set takes effect at the next plan, which it forces.
  void set_closed(const std::vector<Box> & closed);

  bool pose(double & x, double & y, double & yaw) const;

  /// The winning region for a goal from the current knowledge, without moving.
  /// Publishes it, and reports whether the robot stands in it and which bays
  /// the formula lifted. Used by the carrier to show the precondition as a
  /// region before the action it guards is ever dispatched.
  struct Assessment
  {
    bool ok{false};
    bool reachable{false};
    std::size_t region{0};
    std::uint32_t iterations{0};
    std::vector<std::string> lifted;
    std::vector<std::string> known_open;
    std::vector<std::string> route_bays;   ///< bays the extracted route crosses
    double route_length{0.0};
    std::string formula;
    std::string error;
  };
  Assessment assess(double x, double y, double radius, bool publish_route);

  const std::string & model_json() const {return model_json_;}

private:
  void on_odom(nav_msgs::msg::Odometry::SharedPtr msg);
  void on_scan(sensor_msgs::msg::LaserScan::SharedPtr msg);
  void on_known(nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void on_state(std_msgs::msg::String::SharedPtr msg);

  bool blocked_ahead() const;
  void publish_route(const std::vector<std::size_t> & cells);
  void publish_region(const Reach & reach, const SafeSet & safe);

  rclcpp::Node::SharedPtr node_;
  Config config_;
  Grid floorplan_;

  mutable std::mutex lock_;
  bool have_pose_{false};
  double x_{0.0}, y_{0.0}, yaw_{0.0};
  sensor_msgs::msg::LaserScan::SharedPtr scan_;
  Grid known_;
  std::string model_json_;
  std::uint64_t knowledge_version_{0};

  // The route being followed, in metres, and when and against what it was made.
  std::vector<std::pair<double, double>> route_;
  std::size_t route_at_{0};
  rclcpp::Time planned_at_;
  rclcpp::Time blocked_since_;     ///< zero while the laser is not holding it
  rclcpp::Time backing_until_;     ///< zero unless recovering
  std::uint64_t planned_version_{~0ULL};
  double goal_x_{1e9}, goal_y_{1e9};

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr route_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr region_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr known_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_sub_;
  std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> others_subs_;
  std::map<std::string, std::pair<double, double>> others_;
  std::vector<Box> closed_;
};

}  // namespace pass_through

#endif  // PASS_THROUGH_DEMO__DRIVER_HPP_
