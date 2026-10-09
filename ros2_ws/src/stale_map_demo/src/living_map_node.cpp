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

// One robot's map, kept current by its own laser and by the maps it is sent.
//
// It starts as the shift map, every cell known and observed at time zero:
// what every robot is given at the start of the shift. Each scan of the
// robot's laser is then cast into it from the robot's pose: the cells a beam
// crosses are evidence of free space, the cell it ends in evidence of an
// obstacle, each stamped with the time it was observed. Evidence is kept as
// log-odds and a cell changes class only when it crosses a threshold, so one
// beam grazing past a load face does not clear a cell the beams before it
// hit, and one noisy return does not put a load where there is none. Nothing else changes a cell, so a change the
// robot's laser never reaches is a change its map never shows: the map goes
// stale without the robot knowing, which is the static-world assumption every
// occupancy map is built on.
//
// A map is also changed by one it is sent, merged under the receiver's rule
// (epistemic_slam::MergeRule): confidence, overwrite, or recency, the last by
// the observation times the map carries with it.
//
// Two things are kept off the map. The other robots, whose odometry is read
// and whose returns are dropped: a robot crossing a bay is not a load in it.
// And the robot itself, which its own laser does not see.
//
// Interface, in the robot's namespace:
//
//   scan, odom            in: this robot's laser and pose
//   /<other>/odom         in: where every other robot stands
//   known_map             out, latched: the map, for the driver and RViz
//   readings              out, latched: each bay read on the map, JSON
//   offer                 epistemic_msgs/srv/OfferMap: this map of a region
//   receive               epistemic_msgs/srv/ReceiveMap: merge a sent map

#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

#include "epistemic_msgs/srv/offer_map.hpp"
#include "epistemic_msgs/srv/receive_map.hpp"
#include "epistemic_slam/map_fusion.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "pass_through_demo/grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;   // NOLINT(build/namespaces)
using pass_through::Grid;

namespace
{

constexpr std::int8_t FREE = 0;
constexpr std::int8_t OCCUPIED = 100;

// Log-odds evidence. A cell of the shift map, or one taken from another
// robot's map, starts at the prior of its class; a return adds HIT and a beam
// through subtracts MISS, once per scan. A cell is occupied above ON and free
// below OFF, and keeps its class between them. Clearing a load the shift map
// has takes seven scans through it; reading a new one, four returns from it.
constexpr float PRIOR = 2.0f;
constexpr float HIT = 0.7f;
constexpr float MISS = 0.4f;
constexpr float ON = 0.5f;
constexpr float OFF = -0.5f;
constexpr float CEILING = 3.0f;

float prior_of(std::int8_t value)
{
  return value >= 65 ? PRIOR : (value >= 0 && value < 25 ? -PRIOR : 0.0f);
}

// The Waffle's laser sits 6.4 cm behind the centre of its base.
constexpr double LASER_X = -0.064;

double yaw_of(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

}  // namespace

class LivingMap : public rclcpp::Node
{
public:
  LivingMap()
  : rclcpp::Node("living_map")
  {
    agent_ = declare_parameter<std::string>("agent", "r1");
    const auto shift_map = declare_parameter<std::string>("shift_map", "");
    const auto names = declare_parameter<std::vector<std::string>>("bays", std::vector<std::string>{});
    const auto flat = declare_parameter<std::vector<double>>("bay_regions", std::vector<double>{});
    const auto rule = declare_parameter<std::string>("rule", "recency");
    const auto others = declare_parameter<std::vector<std::string>>("others", std::vector<std::string>{});
    range_ = declare_parameter<double>("lidar_range", 12.0);
    mask_ = declare_parameter<double>("mask_radius", 0.45);
    every_ = declare_parameter<int>("scan_every", 4);
    frame_ = declare_parameter<std::string>("frame", "map");

    if (!epistemic_slam::parse_rule(rule, rule_)) {
      throw std::runtime_error("rule must be confidence, overwrite or recency, not " + rule);
    }
    map_ = pass_through::load_map_yaml(shift_map);
    observed_.assign(map_.size(), 0.0);
    odds_.resize(map_.size());
    for (std::size_t i = 0; i < map_.size(); ++i) {odds_[i] = prior_of(map_.cells[i]);}
    touched_.assign(map_.size(), 0);
    bays_ = pass_through::boxes_from(names, flat);
    for (const auto & [name, box] : bays_) {
      cells_[name] = map_.cells_in(box);
      how_[name] = "the shift map";
    }

    const auto latched = rclcpp::QoS(1).transient_local().reliable();
    map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("known_map", latched);
    readings_pub_ = create_publisher<std_msgs::msg::String>("readings", latched);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "odom", rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::SharedPtr m) {
        std::lock_guard<std::mutex> held(lock_);
        x_ = m->pose.pose.position.x;
        y_ = m->pose.pose.position.y;
        yaw_ = yaw_of(m->pose.pose.orientation);
        have_pose_ = true;
      });
    for (const auto & other : others) {
      if (other == agent_) {continue;}
      others_subs_.push_back(create_subscription<nav_msgs::msg::Odometry>(
          "/" + other + "/odom", rclcpp::SensorDataQoS(),
          [this, other](nav_msgs::msg::Odometry::SharedPtr m) {
            std::lock_guard<std::mutex> held(lock_);
            others_[other] = {m->pose.pose.position.x, m->pose.pose.position.y};
          }));
    }
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::LaserScan::SharedPtr m) {on_scan(*m);});

    offer_srv_ = create_service<epistemic_msgs::srv::OfferMap>(
      "offer",
      [this](
        const std::shared_ptr<epistemic_msgs::srv::OfferMap::Request> req,
        std::shared_ptr<epistemic_msgs::srv::OfferMap::Response> res) {offer(*req, *res);});
    receive_srv_ = create_service<epistemic_msgs::srv::ReceiveMap>(
      "receive",
      [this](
        const std::shared_ptr<epistemic_msgs::srv::ReceiveMap::Request> req,
        std::shared_ptr<epistemic_msgs::srv::ReceiveMap::Response> res) {receive(*req, *res);});

    map_timer_ = create_wall_timer(1s, [this]() {publish_map();});
    readings_timer_ = create_wall_timer(500ms, [this]() {publish_readings();});

    RCLCPP_INFO(
      get_logger(), "[map] %s: the shift map, %ux%u cells at %.2f m; merges by %s; "
      "%zu bays watched", agent_.c_str(), map_.width, map_.height, map_.resolution,
      epistemic_slam::to_string(rule_), bays_.size());
  }

private:
  // ─── The laser ────────────────────────────────────────────────────────────

  bool near_other(double x, double y) const
  {
    for (const auto & [ns, at] : others_) {
      (void)ns;
      if (std::hypot(x - at.first, y - at.second) < mask_) {return true;}
    }
    return false;
  }

  /// One observation of cell `i` in this scan, at most one per cell.
  void observe(std::size_t i, bool hit, double stamp)
  {
    if (touched_[i] == scan_mark_) {return;}
    touched_[i] = scan_mark_;
    odds_[i] = std::clamp(odds_[i] + (hit ? HIT : -MISS), -CEILING, CEILING);
    observed_[i] = stamp;
    if (odds_[i] > ON) {
      map_.cells[i] = OCCUPIED;
    } else if (odds_[i] < OFF) {
      map_.cells[i] = FREE;
    }
  }

  /// The cells from (x0, y0) along `angle` for `length` metres, as free space.
  void free_along(double x0, double y0, double angle, double length, double stamp)
  {
    const double step = map_.resolution * 0.5;
    const double c = std::cos(angle), s = std::sin(angle);
    for (double d = 0.0; d < length; d += step) {
      std::size_t i;
      if (!map_.index(x0 + d * c, y0 + d * s, i)) {return;}
      observe(i, false, stamp);
    }
  }

  void on_scan(const sensor_msgs::msg::LaserScan & scan)
  {
    if (++scans_ % static_cast<std::size_t>(std::max(1, every_)) != 0) {return;}
    std::lock_guard<std::mutex> held(lock_);
    if (!have_pose_) {return;}
    const double stamp = rclcpp::Time(scan.header.stamp).seconds();
    const double sx = x_ + LASER_X * std::cos(yaw_);
    const double sy = y_ + LASER_X * std::sin(yaw_);
    const double reach = std::min(static_cast<double>(scan.range_max), range_);

    // Returns first, so that a beam grazing past a load does not count as
    // free space in the cell the next beam ended in.
    if (++scan_mark_ == 0) {
      std::fill(touched_.begin(), touched_.end(), 0);
      scan_mark_ = 1;
    }
    std::vector<std::pair<double, double>> beams;   // angle, free length
    beams.reserve(scan.ranges.size());
    for (std::size_t k = 0; k < scan.ranges.size(); ++k) {
      const double angle = yaw_ + scan.angle_min + k * scan.angle_increment;
      const float r = scan.ranges[k];
      if (std::isfinite(r) && r < scan.range_min) {continue;}
      if (!std::isfinite(r) || r >= reach) {
        beams.emplace_back(angle, reach);
        continue;
      }
      const double hx = sx + r * std::cos(angle);
      const double hy = sy + r * std::sin(angle);
      if (near_other(hx, hy)) {
        // Another robot. The floor behind it was not seen, and it is not a
        // part of the floor.
        beams.emplace_back(angle, std::max(0.0, r - mask_));
        continue;
      }
      std::size_t i;
      if (map_.index(hx, hy, i)) {observe(i, true, stamp);}
      beams.emplace_back(angle, r - map_.resolution * 0.5);
    }
    // Each beam is a cone of one angular step; three rays across it, so that
    // at twelve metres no cell falls between two beams unobserved.
    const double third = scan.angle_increment / 3.0;
    for (const auto & [angle, length] : beams) {
      for (const double off : {-third, 0.0, third}) {
        free_along(sx, sy, angle + off, length, stamp);
      }
    }
    dirty_ = true;
  }

  // ─── Readings ─────────────────────────────────────────────────────────────

  std::string verdict(const std::string & bay) const
  {
    return pass_through::to_string(pass_through::read_region(map_, bays_.at(bay)).verdict);
  }

  void publish_readings()
  {
    std::lock_guard<std::mutex> held(lock_);
    nlohmann::json out;
    out["agent"] = agent_;
    out["rule"] = epistemic_slam::to_string(rule_);
    out["stamp"] = now().seconds();
    for (const auto & [name, box] : bays_) {
      const auto r = pass_through::read_region(map_, box);
      const std::string v = pass_through::to_string(r.verdict);
      double newest = 0.0;
      for (const auto i : cells_.at(name)) {newest = std::max(newest, observed_[i]);}
      out["bays"][name] = {
        {"verdict", v}, {"cells", r.cells}, {"free", r.free}, {"occupied", r.occupied},
        {"unknown", r.unknown}, {"newest", newest}, {"how", how_[name]}};
      if (last_[name] != v) {
        // A reading changes because a merge changed it, which says so, or
        // because the laser did.
        const auto cause = cause_.find(name);
        how_[name] = cause != cause_.end() ? cause->second : "its own laser";
        if (cause != cause_.end()) {cause_.erase(cause);}
        if (!last_[name].empty()) {
          RCLCPP_INFO(
            get_logger(), "[map] %s reads %s %s, was %s: from %s; %zu of %zu cells occupied",
            agent_.c_str(), name.c_str(), v.c_str(), last_[name].c_str(), how_[name].c_str(),
            r.occupied, r.cells);
        } else {
          how_[name] = "the shift map";
        }
        out["bays"][name]["how"] = how_[name];
        last_[name] = v;
      }
    }
    std_msgs::msg::String msg;
    msg.data = out.dump();
    readings_pub_->publish(msg);
  }

  void publish_map()
  {
    std::lock_guard<std::mutex> held(lock_);
    auto msg = pass_through::to_msg(map_, frame_);
    msg.header.stamp = now();
    map_pub_->publish(msg);
    dirty_ = false;
  }

  // ─── Exchange ─────────────────────────────────────────────────────────────

  std::vector<std::size_t> region(const std::string & name, bool & ok) const
  {
    ok = true;
    if (name.empty()) {
      std::vector<std::size_t> all(map_.size());
      for (std::size_t i = 0; i < all.size(); ++i) {all[i] = i;}
      return all;
    }
    const auto it = cells_.find(name);
    if (it == cells_.end()) {
      ok = false;
      return {};
    }
    return it->second;
  }

  void offer(
    const epistemic_msgs::srv::OfferMap::Request & req,
    epistemic_msgs::srv::OfferMap::Response & res)
  {
    std::lock_guard<std::mutex> held(lock_);
    bool ok;
    const auto cells = region(req.region, ok);
    if (!ok) {
      res.ok = false;
      res.message = agent_ + " watches no region " + req.region;
      return;
    }
    Grid part = pass_through::blank_like(map_, -1);
    res.observed.assign(map_.size(), 0.0);
    for (const auto i : cells) {
      part.cells[i] = map_.cells[i];
      res.observed[i] = observed_[i];
    }
    res.map = pass_through::to_msg(part, frame_);
    res.map.header.stamp = now();
    res.verdict = req.region.empty() ? "" : verdict(req.region);
    res.ok = true;
    res.message = agent_ + "'s map of " + (req.region.empty() ? "the floor" : req.region);
  }

  void receive(
    const epistemic_msgs::srv::ReceiveMap::Request & req,
    epistemic_msgs::srv::ReceiveMap::Response & res)
  {
    std::lock_guard<std::mutex> held(lock_);
    res.rule = epistemic_slam::to_string(rule_);
    const Grid incoming = pass_through::from_msg(req.map);
    if (!incoming.same_geometry(map_)) {
      res.ok = false;
      res.message = "the map from " + req.source + " is not on this robot's grid";
      return;
    }
    bool ok;
    const auto cells = region(req.region, ok);
    if (!ok) {
      res.ok = false;
      res.message = agent_ + " watches no region " + req.region;
      return;
    }
    if (!req.region.empty()) {res.verdict_before = verdict(req.region);}
    const auto before = map_.cells;
    const auto merge = epistemic_slam::merge_cells(
      map_.cells, observed_, incoming.cells, req.observed, cells, rule_);
    // A reading taken from another map is held with the evidence of a
    // reading of the shift map, no more and no less.
    for (const auto i : cells) {
      if (map_.cells[i] != before[i]) {odds_[i] = prior_of(map_.cells[i]);}
    }
    res.learned = static_cast<std::uint32_t>(merge.learned);
    res.changed = static_cast<std::uint32_t>(merge.changed);
    res.kept = static_cast<std::uint32_t>(merge.kept);
    if (!req.region.empty()) {
      res.verdict_after = verdict(req.region);
      if (res.verdict_after != res.verdict_before) {
        cause_[req.region] = "a map from " + req.source;
      }
    } else {
      for (const auto & [name, box] : bays_) {
        (void)box;
        cause_[name] = "a map from " + req.source;
      }
    }
    res.ok = true;
    res.message = agent_ + " merged " + req.source + "'s map" +
      (req.region.empty() ? "" : " of " + req.region) + " by " + res.rule;
    RCLCPP_INFO(
      get_logger(), "[map] %s merged %s's map of %s by %s: %zu cells considered, %zu learned, "
      "%zu replaced, %zu kept against it; %s -> %s", agent_.c_str(), req.source.c_str(),
      req.region.empty() ? "the floor" : req.region.c_str(), res.rule.c_str(), merge.considered,
      merge.learned, merge.changed, merge.kept, res.verdict_before.c_str(),
      res.verdict_after.c_str());
    dirty_ = true;
  }

  std::string agent_;
  std::string frame_;
  epistemic_slam::MergeRule rule_{epistemic_slam::MergeRule::Recency};
  double range_{12.0};
  double mask_{0.45};
  int every_{4};
  std::size_t scans_{0};

  Grid map_;
  std::vector<double> observed_;
  std::vector<float> odds_;
  std::vector<std::uint32_t> touched_;
  std::uint32_t scan_mark_{0};
  std::map<std::string, pass_through::Box> bays_;
  std::map<std::string, std::vector<std::size_t>> cells_;
  std::map<std::string, std::string> last_;
  std::map<std::string, std::string> how_;
  std::map<std::string, std::string> cause_;
  bool dirty_{true};

  bool have_pose_{false};
  double x_{0.0}, y_{0.0}, yaw_{0.0};
  std::map<std::string, std::pair<double, double>> others_;
  std::mutex lock_;

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr readings_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> others_subs_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Service<epistemic_msgs::srv::OfferMap>::SharedPtr offer_srv_;
  rclcpp::Service<epistemic_msgs::srv::ReceiveMap>::SharedPtr receive_srv_;
  rclcpp::TimerBase::SharedPtr map_timer_;
  rclcpp::TimerBase::SharedPtr readings_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LivingMap>());
  rclcpp::shutdown();
  return 0;
}
