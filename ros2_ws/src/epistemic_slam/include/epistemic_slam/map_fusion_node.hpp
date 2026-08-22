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

#ifndef EPISTEMIC_SLAM__MAP_FUSION_NODE_HPP_
#define EPISTEMIC_SLAM__MAP_FUSION_NODE_HPP_

#include <memory>
#include <optional>
#include <string>

#include "epistemic_slam/map_fusion.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace epistemic_slam
{

/**
 * @class epistemic_slam::MapFusionNode
 * @brief Reconciles two robots' maps when their link comes back.
 *
 * The fleet spends most of its time disconnected, each robot mapping what it
 * can see. Fusion is an event, not a loop: it happens when a link is restored,
 * and the interesting output is not the merged map but the difference, because
 * the difference is what each robot has just learned and therefore what the
 * epistemic state has to be told about.
 *
 * That is why this is a service and not a timer. Running continuously would
 * report the same "newly known" cells over and over and make the epistemic
 * layer's history meaningless.
 *
 * Interface:
 *
 *   ~/map_a, ~/map_b   nav_msgs/OccupancyGrid, transient local. The two maps
 *                      to reconcile. Latched because a map published while
 *                      this node was down is still the current map.
 *   ~/merged           nav_msgs/OccupancyGrid, transient local. Published only
 *                      when a fusion succeeds.
 *   ~/fuse             std_srvs/Trigger. Run it now. The response message
 *                      carries what changed, or why it could not be done.
 *
 * Parameters:
 *
 *   free_below         Occupancy below this is free. Default 25.
 *   occupied_above     Occupancy above this is an obstacle. Default 65.
 *                      Between them a cell has been seen without being
 *                      settled, and is treated as unknown.
 *
 * Thesis section 3.12.2 and CU-03.
 */
class MapFusionNode : public rclcpp::Node
{
public:
  explicit MapFusionNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /// Reconcile the two maps held right now.
  ///
  /// Separate from the service callback so it can be driven directly from a
  /// test, and so the caller can see the whole result rather than the sentence
  /// the service reduces it to. Nullopt when a map is still missing.
  std::optional<Fusion> fuse_current() const;

private:
  void on_map_a(nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void on_map_b(nav_msgs::msg::OccupancyGrid::SharedPtr msg);

  void fuse_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  /// The thresholds as the parameters currently say.
  Thresholds thresholds() const;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_a_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_b_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr merged_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr fuse_service_;

  nav_msgs::msg::OccupancyGrid::SharedPtr map_a_;
  nav_msgs::msg::OccupancyGrid::SharedPtr map_b_;
};

}  // namespace epistemic_slam

#endif  // EPISTEMIC_SLAM__MAP_FUSION_NODE_HPP_
