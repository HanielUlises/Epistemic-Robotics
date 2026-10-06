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

// One robot's map, read as what that robot knows.
//
// Two grids are kept, and the difference between them is the point of the
// node. `own_map` is the robot's SLAM map placed on the floor plan's grid:
// what this robot has observed with its own laser, and nothing else. It is
// what a sensing action is read from, because a sensing action is an agent
// looking. `known_map` is `own_map` fused with every map this robot has been
// sent: what it knows, however it came to know it. It is what the robot
// navigates by.
//
// A map is sent by calling `absorb` with it. The fusion is epistemic_slam's,
// unchanged, and the reply says what the exchange actually carried -- cells
// newly known, over the grid and inside each bay -- because the announcement
// the executor applies to the model afterwards has to be checkable against
// the map the receiver now holds.
//
// Interface, in the robot's namespace:
//
//   map         nav_msgs/OccupancyGrid in, transient local: this robot's SLAM
//   own_map     nav_msgs/OccupancyGrid out, latched
//   known_map   nav_msgs/OccupancyGrid out, latched
//   readings    std_msgs/String out, latched: each bay read on both maps, JSON
//   absorb      epistemic_msgs/srv/AbsorbMap

#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "epistemic_msgs/srv/absorb_map.hpp"
#include "epistemic_slam/map_fusion.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "pass_through_demo/grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;   // NOLINT(build/namespaces)
using pass_through::Grid;

class KnowledgeMap : public rclcpp::Node
{
public:
  KnowledgeMap()
  : rclcpp::Node("knowledge_map")
  {
    agent_ = declare_parameter<std::string>("agent", "west");
    const auto floorplan = declare_parameter<std::string>("floorplan", "");
    const auto names = declare_parameter<std::vector<std::string>>("bays", std::vector<std::string>{});
    const auto flat = declare_parameter<std::vector<double>>("bay_boxes", std::vector<double>{});
    frame_ = declare_parameter<std::string>("frame", "map");

    geometry_ = pass_through::blank_like(pass_through::load_map_yaml(floorplan), -1);
    bays_ = pass_through::boxes_from(names, flat);
    received_ = geometry_;
    own_ = geometry_;

    const auto latched = rclcpp::QoS(1).transient_local().reliable();
    own_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("own_map", latched);
    known_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("known_map", latched);
    readings_pub_ = create_publisher<std_msgs::msg::String>("readings", latched);

    slam_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "map", latched,
      [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        std::lock_guard<std::mutex> held(lock_);
        own_ = pass_through::resample(pass_through::from_msg(*msg), geometry_);
        dirty_ = true;
      });

    absorb_srv_ = create_service<epistemic_msgs::srv::AbsorbMap>(
      "absorb",
      [this](
        const std::shared_ptr<epistemic_msgs::srv::AbsorbMap::Request> request,
        std::shared_ptr<epistemic_msgs::srv::AbsorbMap::Response> response) {
        absorb(*request, *response);
      });

    // Published on a clock and not per SLAM update, so a latched reader that
    // subscribes late still gets a recent map, and so the three robots' maps
    // do not arrive at RViz in bursts.
    timer_ = create_wall_timer(1s, [this]() {publish(false);});

    RCLCPP_INFO(
      get_logger(), "[knowledge] %s: %ux%u cells at %.2f m, watching %zu bays",
      agent_.c_str(), geometry_.width, geometry_.height, geometry_.resolution, bays_.size());
  }

private:
  /// own fused with received, on the common grid.
  Grid known() const
  {
    const auto fused = epistemic_slam::fuse(
      pass_through::to_msg(own_, frame_), pass_through::to_msg(received_, frame_));
    return fused.ok ? pass_through::from_msg(fused.merged) : own_;
  }

  void absorb(
    const epistemic_msgs::srv::AbsorbMap::Request & request,
    epistemic_msgs::srv::AbsorbMap::Response & response)
  {
    std::lock_guard<std::mutex> held(lock_);
    const Grid incoming = pass_through::from_msg(request.map);
    if (!incoming.same_geometry(geometry_)) {
      response.ok = false;
      response.message = "the map from " + request.source + " is not on this robot's grid";
      RCLCPP_ERROR(get_logger(), "[knowledge] %s: %s", agent_.c_str(), response.message.c_str());
      return;
    }

    const Grid before = known();
    const auto fused = epistemic_slam::fuse(
      pass_through::to_msg(received_, frame_), request.map);
    if (!fused.ok) {
      response.ok = false;
      response.message = fused.error;
      return;
    }
    received_ = pass_through::from_msg(fused.merged);
    const Grid after = known();

    std::size_t newly = 0;
    std::vector<char> gained(after.size(), 0);
    for (std::size_t i = 0; i < after.size(); ++i) {
      if (!epistemic_slam::is_known(before.cells[i]) && epistemic_slam::is_known(after.cells[i])) {
        gained[i] = 1;
        ++newly;
      }
    }
    response.ok = true;
    response.newly_known = static_cast<std::uint32_t>(newly);
    response.conflicts = static_cast<std::uint32_t>(fused.conflicts.size());

    std::string per_bay;
    for (const auto & [name, box] : bays_) {
      std::uint32_t count = 0;
      for (const auto i : after.cells_in(box)) {
        count += gained[i] ? 1 : 0;
      }
      response.regions.push_back(name);
      response.newly_known_in_region.push_back(count);
      per_bay += " " + name + "=" + std::to_string(count);
    }
    response.message = std::to_string(newly) + " cells newly known";

    RCLCPP_INFO(
      get_logger(), "[knowledge] %s absorbed the map of %s: %zu cells newly known, "
      "%zu conflicts; in the bays:%s",
      agent_.c_str(), request.source.c_str(), newly, fused.conflicts.size(), per_bay.c_str());
    dirty_ = true;
    publish_locked(true);
  }

  void publish(bool force)
  {
    std::lock_guard<std::mutex> held(lock_);
    publish_locked(force);
  }

  void publish_locked(bool force)
  {
    if (!dirty_ && !force) {
      return;
    }
    dirty_ = false;
    const Grid knows = known();
    const auto stamp = now();

    auto own_msg = pass_through::to_msg(own_, frame_);
    own_msg.header.stamp = stamp;
    own_pub_->publish(own_msg);
    auto known_msg = pass_through::to_msg(knows, frame_);
    known_msg.header.stamp = stamp;
    known_pub_->publish(known_msg);

    nlohmann::json readings;
    readings["agent"] = agent_;
    std::size_t own_cells = 0, known_cells = 0;
    for (std::size_t i = 0; i < own_.size(); ++i) {
      own_cells += epistemic_slam::is_known(own_.cells[i]) ? 1 : 0;
      known_cells += epistemic_slam::is_known(knows.cells[i]) ? 1 : 0;
    }
    readings["own_cells"] = own_cells;
    readings["known_cells"] = known_cells;
    for (const auto & [name, box] : bays_) {
      const auto own = pass_through::read_region(own_, box);
      const auto kn = pass_through::read_region(knows, box);
      readings["own"][name] = {
        {"verdict", pass_through::to_string(own.verdict)}, {"cells", own.cells},
        {"free", own.free}, {"occupied", own.occupied}, {"unknown", own.unknown}};
      readings["known"][name] = {
        {"verdict", pass_through::to_string(kn.verdict)}, {"cells", kn.cells},
        {"free", kn.free}, {"occupied", kn.occupied}, {"unknown", kn.unknown}};

      // Said once, when it changes: the instant a robot's map first decides a
      // bay is worth a line in the log.
      const std::string key = std::string(pass_through::to_string(own.verdict)) + "/" +
        pass_through::to_string(kn.verdict);
      if (last_[name] != key) {
        last_[name] = key;
        RCLCPP_INFO(
          get_logger(), "[knowledge] %s reads %s: own map %s (%zu of %zu cells seen, %zu occupied), "
          "known map %s", agent_.c_str(), name.c_str(), pass_through::to_string(own.verdict),
          own.cells - own.unknown, own.cells, own.occupied, pass_through::to_string(kn.verdict));
      }
    }
    std_msgs::msg::String text;
    text.data = readings.dump();
    readings_pub_->publish(text);
  }

  std::string agent_;
  std::string frame_;
  Grid geometry_;
  Grid own_;
  Grid received_;
  std::map<std::string, pass_through::Box> bays_;
  std::map<std::string, std::string> last_;
  bool dirty_{true};
  std::mutex lock_;

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr own_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr known_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr readings_pub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr slam_sub_;
  rclcpp::Service<epistemic_msgs::srv::AbsorbMap>::SharedPtr absorb_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KnowledgeMap>());
  rclcpp::shutdown();
  return 0;
}
