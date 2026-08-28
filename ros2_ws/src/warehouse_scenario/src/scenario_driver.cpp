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
//
// Drives the running planner node through every case of the warehouse
// scenario.
//
// It also stands in for a producer the stack is missing. The planner
// subscribes to /epistemic/state expecting a Kripke snapshot, and nothing
// publishes one yet: the epistemic state node of eplansys latches the *shape*
// of its model and its goal, not the model itself. Until it does, the
// snapshot comes from the scenario and this node puts it on the wire, which
// is exactly the seam a snapshot publisher would fill.
//
// One process serves every case, and each case latches its own map and its
// own snapshot: a planner that answered the second query with the first
// query's map would pass a check that restarted between cases.

#include <chrono>
#include <filesystem>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/string.hpp>

#include "epistemic_msgs/msg/mu_path_query.hpp"
#include "scenario_io.hpp"
#include "warehouse_scenario/warehouse.hpp"

namespace fs = std::filesystem;
using namespace std::chrono_literals;
using namespace warehouse_scenario;

namespace
{

/// Latched, so the planner reads the map and the state whether it came up
/// before this driver or after it.
rclcpp::QoS latched()
{
  return rclcpp::QoS(1).transient_local().reliable();
}

class ScenarioDriver : public rclcpp::Node
{
public:
  explicit ScenarioDriver(fs::path root)
  : Node("warehouse_scenario_driver"), root_(std::move(root))
  {
    map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/map", latched());
    state_pub_ = create_publisher<std_msgs::msg::String>("/epistemic/state", latched());
    query_pub_ = create_publisher<epistemic_msgs::msg::MuPathQuery>(
      "/mu_planner/query", 10);

    // Only the answer to the question this driver asked counts: the planner
    // stamps a path with the stamp of the query it answers, and a path with
    // any other stamp belongs to somebody else's question.
    status_sub_ = create_subscription<std_msgs::msg::String>(
      "/mu_planner/status", rclcpp::QoS(1).transient_local(),
      [this](const std_msgs::msg::String::SharedPtr msg) {status_ = msg->data;});

    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      "/mu_planner/path", 10,
      [this](const nav_msgs::msg::Path::SharedPtr msg) {
        if (answers_us(msg->header.stamp)) {path_ = msg;}
      });
    sensing_sub_ = create_subscription<nav_msgs::msg::Path>(
      "/mu_planner/sensing", 10,
      [this](const nav_msgs::msg::Path::SharedPtr msg) {
        if (answers_us(msg->header.stamp)) {sensing_ = msg;}
      });
  }

  /// @return the number of cases the planner did not answer.
  int run_all(std::chrono::seconds timeout)
  {
    if (!wait_for_planner(timeout)) {
      RCLCPP_ERROR(get_logger(), "no planner is listening on /mu_planner/query");
      return -1;
    }

    int unanswered = 0;
    for (const auto & c : cases()) {
      unanswered += run_case(c, timeout) ? 0 : 1;
    }
    return unanswered;
  }

private:
  bool answers_us(const builtin_interfaces::msg::Time & stamp) const
  {
    return stamp.sec == asked_.sec && stamp.nanosec == asked_.nanosec;
  }

  bool wait_for_planner(std::chrono::seconds timeout)
  {
    const auto deadline = now() + rclcpp::Duration(timeout);
    while (rclcpp::ok() && now() < deadline) {
      if (query_pub_->get_subscription_count() > 0 &&
        map_pub_->get_subscription_count() > 0)
      {
        return true;
      }
      rclcpp::spin_some(get_node_base_interface());
      rclcpp::sleep_for(50ms);
    }
    return false;
  }

  /// Publishes the case's map and snapshot, and returns the two content
  /// hashes the planner will report once it has actually taken them.
  std::pair<uint64_t, uint64_t> publish_inputs(const Case & c)
  {
    const auto grid = build_grid(c.stage);

    nav_msgs::msg::OccupancyGrid map;
    map.header.frame_id = "map";
    map.header.stamp = now();
    map.info.resolution = static_cast<float>(kResolution);
    map.info.width = kWidth;
    map.info.height = kHeight;
    map.info.origin.position.x = kOriginX;
    map.info.origin.position.y = kOriginY;
    map.info.origin.orientation.w = 1.0;
    map.data.assign(grid.begin(), grid.end());
    map_pub_->publish(map);

    std_msgs::msg::String state;
    state.data = snapshot_for(c);
    state_pub_->publish(state);

    return {
      mu_path_planner::content_hash(map.data.data(), map.data.size()),
      mu_path_planner::content_hash(state.data)
    };
  }

  /// Waits until the planner reports that it is holding this case's map and
  /// this case's snapshot. Without it the query races the two callbacks that
  /// deliver them, and the planner answers with the case before -- which it
  /// does quietly, since answering with a stale map is not an error it can
  /// detect from the inside.
  bool wait_for_inputs(uint64_t map_hash, uint64_t state_hash,
    std::chrono::seconds timeout)
  {
    const std::string want_map = "\"map_hash\": " + std::to_string(map_hash);
    const std::string want_state = "\"state_hash\": " + std::to_string(state_hash);

    const auto deadline = now() + rclcpp::Duration(timeout);
    while (rclcpp::ok() && now() < deadline) {
      if (status_.find(want_map) != std::string::npos &&
        status_.find(want_state) != std::string::npos)
      {
        return true;
      }
      rclcpp::spin_some(get_node_base_interface());
      rclcpp::sleep_for(10ms);
    }
    return false;
  }

  bool run_case(const Case & c, std::chrono::seconds timeout)
  {
    path_.reset();
    sensing_.reset();

    const auto [map_hash, state_hash] = publish_inputs(c);
    if (!wait_for_inputs(map_hash, state_hash, timeout)) {
      RCLCPP_ERROR(get_logger(), "%-24s planner never reported this map and state",
        c.name.c_str());
      return false;
    }

    epistemic_msgs::msg::MuPathQuery query;
    query.header.frame_id = "map";
    query.header.stamp = now();
    asked_ = query.header.stamp;
    query.agent_id = c.agent_id;
    query.goal_zone = c.goal_zone;
    query.safety_formula_json = c.safety_formula_json;
    query.require_epistemic_goal = c.require_epistemic_goal;
    query_pub_->publish(query);

    const auto deadline = now() + rclcpp::Duration(timeout);
    while (rclcpp::ok() && now() < deadline && !(path_ && sensing_)) {
      rclcpp::spin_some(get_node_base_interface());
      rclcpp::sleep_for(20ms);
    }

    const bool answered = static_cast<bool>(path_) && static_cast<bool>(sensing_);
    Route route;
    if (answered) {
      for (const auto & pose : path_->poses) {
        route.path.push_back(Point{pose.pose.position.x, pose.pose.position.y});
      }
      for (const auto & pose : sensing_->poses) {
        route.sensing.push_back(Point{pose.pose.position.x, pose.pose.position.y});
      }
    }

    // The same case, answered here with no ROS in the way. If the two differ
    // the wire changed the answer, and that is worth failing over: everything
    // between them -- the float32 resolution of a map, a second planner on the
    // graph, a query racing the map it was meant for -- has already done it
    // once each.
    const Outcome reference = run(c);
    const Route expected = route_of(reference);
    const bool agrees = answered &&
      expected.path.size() == route.path.size() &&
      expected.sensing.size() == route.sensing.size();

    nlohmann::json extra;
    extra["answered"] = answered;
    extra["offline_path_length"] = expected.path.size();
    extra["offline_sensing_length"] = expected.sensing.size();
    extra["agrees_with_offline"] = agrees;
    write_result(root_ / "out", c.name + ".ros", to_json(c, route, "ros", extra));

    if (!answered) {
      RCLCPP_ERROR(get_logger(), "%-24s TIMED OUT", c.name.c_str());
    } else if (!agrees) {
      RCLCPP_ERROR(get_logger(),
        "%-24s over ROS path=%zu sensing=%zu, offline path=%zu sensing=%zu",
        c.name.c_str(), route.path.size(), route.sensing.size(),
        expected.path.size(), expected.sensing.size());
    } else {
      RCLCPP_INFO(get_logger(), "%-24s answered path=%zu sensing=%zu, as offline",
        c.name.c_str(), route.path.size(), route.sensing.size());
    }
    return answered && agrees;
  }

  fs::path root_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<epistemic_msgs::msg::MuPathQuery>::SharedPtr query_pub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sensing_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
  std::string status_;
  builtin_interfaces::msg::Time asked_;
  nav_msgs::msg::Path::SharedPtr path_;
  nav_msgs::msg::Path::SharedPtr sensing_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  const fs::path root = argc > 1 ? fs::path(argv[1]) : fs::path("scenarios/warehouse");
  auto driver = std::make_shared<ScenarioDriver>(root);

  const int unanswered = driver->run_all(20s);

  rclcpp::shutdown();
  if (unanswered < 0) {return 2;}
  if (unanswered > 0) {
    std::cerr << unanswered << " case(s) went unanswered\n";
    return 1;
  }
  return 0;
}
