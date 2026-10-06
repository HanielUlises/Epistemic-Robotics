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

#ifndef FALSE_BELIEF_DEMO__CRATE_HPP_
#define FALSE_BELIEF_DEMO__CRATE_HPP_

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

#include "gazebo_msgs/srv/set_entity_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"

namespace false_belief
{

/// Bay name -> (x, y), from a flat list of names and a flat list of pairs.
inline std::map<std::string, std::pair<double, double>> bays_from(
  const std::vector<std::string> & names, const std::vector<double> & flat)
{
  std::map<std::string, std::pair<double, double>> out;
  for (std::size_t i = 0; i < names.size() && 2 * i + 1 < flat.size(); ++i) {
    out[names[i]] = {flat[2 * i], flat[2 * i + 1]};
  }
  return out;
}

/// The crate in Gazebo. The robots have no gripper and no lift: a robot that
/// takes the crate carries it on its top plate, which the simulator draws by
/// moving the crate with the robot through gazebo_ros_state. Where the crate
/// is goes out on /false_belief/crate, for the view.
class Crate
{
public:
  Crate(rclcpp::Node::SharedPtr node, double height, double carry_z)
  : height_(height), carry_z_(carry_z)
  {
    client_ = node->create_client<gazebo_msgs::srv::SetEntityState>("/gazebo/set_entity_state");
    pub_ = node->create_publisher<std_msgs::msg::String>(
      "/false_belief/crate", rclcpp::QoS(1).transient_local().reliable());
  }

  /// Ride on a robot standing at (x, y) with heading yaw.
  void carry(const std::string & agent, double x, double y, double yaw)
  {
    set(x, y, carry_z_, yaw);
    announce({{"carried_by", agent}, {"x", x}, {"y", y}});
  }

  /// Stand on the floor of a bay, or at the drop.
  void rest(const std::string & where, double x, double y)
  {
    set(x, y, height_ / 2.0, 0.0);
    announce({{"at", where}, {"x", x}, {"y", y}});
  }

private:
  void set(double x, double y, double z, double yaw)
  {
    if (!client_->service_is_ready()) {
      return;
    }
    auto request = std::make_shared<gazebo_msgs::srv::SetEntityState::Request>();
    request->state.name = "crate";
    request->state.pose.position.x = x;
    request->state.pose.position.y = y;
    request->state.pose.position.z = z;
    request->state.pose.orientation.z = std::sin(yaw / 2.0);
    request->state.pose.orientation.w = std::cos(yaw / 2.0);
    request->state.reference_frame = "world";
    client_->async_send_request(request);
  }

  void announce(const nlohmann::json & j)
  {
    const auto text = j.dump();
    if (text == last_) {
      return;
    }
    last_ = text;
    std_msgs::msg::String m;
    m.data = text;
    pub_->publish(m);
  }

  double height_;
  double carry_z_;
  std::string last_;
  rclcpp::Client<gazebo_msgs::srv::SetEntityState>::SharedPtr client_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
};

/// The shortest range a robot's laser reads within a few beams of straight
/// ahead; infinity when no beam returns, negative when there is no scan yet.
inline double ahead(const sensor_msgs::msg::LaserScan::SharedPtr & scan, int half_width = 3)
{
  if (!scan || scan->ranges.empty()) {
    return -1.0;
  }
  const int n = static_cast<int>(scan->ranges.size());
  const int centre = static_cast<int>(std::lround((0.0 - scan->angle_min) / scan->angle_increment));
  double nearest = std::numeric_limits<double>::infinity();
  for (int k = -half_width; k <= half_width; ++k) {
    const int i = ((centre + k) % n + n) % n;
    const float r = scan->ranges[i];
    if (std::isfinite(r) && r >= scan->range_min && r <= scan->range_max) {
      nearest = std::min(nearest, static_cast<double>(r));
    }
  }
  return nearest;
}

}  // namespace false_belief

#endif  // FALSE_BELIEF_DEMO__CRATE_HPP_
