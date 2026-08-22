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

#include "epistemic_slam/map_fusion_node.hpp"

#include <memory>
#include <optional>
#include <string>

namespace epistemic_slam
{

namespace
{

/// A map published while this node was down is still the current map, so the
/// subscriptions latch. Depth one: only the newest map of each robot matters.
rclcpp::QoS map_qos()
{
  return rclcpp::QoS(1).transient_local().reliable();
}

}  // namespace

MapFusionNode::MapFusionNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("map_fusion", options)
{
  declare_parameter<int>("free_below", 25);
  declare_parameter<int>("occupied_above", 65);

  map_a_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    "~/map_a", map_qos(),
    std::bind(&MapFusionNode::on_map_a, this, std::placeholders::_1));

  map_b_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    "~/map_b", map_qos(),
    std::bind(&MapFusionNode::on_map_b, this, std::placeholders::_1));

  merged_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("~/merged", map_qos());

  fuse_service_ = create_service<std_srvs::srv::Trigger>(
    "~/fuse",
    std::bind(
      &MapFusionNode::fuse_callback, this,
      std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(get_logger(), "waiting for both maps before a fusion can be run");
}

Thresholds MapFusionNode::thresholds() const
{
  Thresholds t;
  t.free_below = static_cast<std::int8_t>(get_parameter("free_below").as_int());
  t.occupied_above = static_cast<std::int8_t>(get_parameter("occupied_above").as_int());
  return t;
}

void MapFusionNode::on_map_a(nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  map_a_ = std::move(msg);
}

void MapFusionNode::on_map_b(nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  map_b_ = std::move(msg);
}

std::optional<Fusion> MapFusionNode::fuse_current() const
{
  if (!map_a_ || !map_b_) {
    return std::nullopt;
  }
  return fuse(*map_a_, *map_b_, thresholds());
}

void MapFusionNode::fuse_callback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;

  const auto result = fuse_current();
  if (!result) {
    response->success = false;
    response->message =
      std::string("no fusion yet: ") +
      (map_a_ ? "map_b has not been received" : "map_a has not been received");
    RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
    return;
  }

  if (!result->ok) {
    // Refusing beats merging two maps that describe different areas: the
    // result would be confidently wrong rather than obviously missing.
    response->success = false;
    response->message = result->error;
    RCLCPP_ERROR(get_logger(), "fusion refused: %s", result->error.c_str());
    return;
  }

  merged_pub_->publish(result->merged);

  response->success = true;
  response->message =
    "a learned " + std::to_string(result->newly_known_to_a.size()) +
    " cells, b learned " + std::to_string(result->newly_known_to_b.size()) +
    ", " + std::to_string(result->conflicts.size()) + " in conflict";

  RCLCPP_INFO(get_logger(), "%s", response->message.c_str());

  if (!result->conflicts.empty()) {
    // Not a failure. Two robots that observed the same cell differently is a
    // reason to go and look again, and the fleet cannot decide that here.
    RCLCPP_WARN(
      get_logger(),
      "%zu cells where one robot saw free and the other saw an obstacle",
      result->conflicts.size());
  }

  // TT-II: announce newly_known_to_a and newly_known_to_b to the epistemic
  // state. Cells are not atoms, so this needs the region definitions and the
  // translation of thesis 3.7, which lives in eplansys as
  // plansys2_epistemic_perception.
}

}  // namespace epistemic_slam
