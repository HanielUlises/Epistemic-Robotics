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

// What a robot that stands still has seen of the carrier, from its own laser.
//
// One instance for the ally r2 in its booth and one for the watcher r3 at its
// post. Neither acts, and neither is told anything by the planner or the
// executor: this node reads the robot's own scan, places the returns in the
// world with the robot's own odometry, and records, region by region, when it
// saw something there. The regions are free floor -- the two gates, the two
// window spots, the two halls, the two bays -- so a return inside one is the
// carrier. It also listens to the carrier's lamp, and counts the lamp as seen
// only while its laser sees the carrier at a window: a lamp is light, and light
// travels as the laser does.
//
// What the robot can conclude is written beside what it saw: the bay, if it
// saw the carrier in a gate or in a bay, or saw the lamp; nothing otherwise.
// That conclusion is the measurement the run is judged by, independent of the
// planner's model. Writes <dir>/observer_<robot>.json at every change and
// <dir>/observer_<robot>.csv, one row per sighting begun or ended.

#include <cmath>
#include <cstdio>
#include <fstream>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"

class Observer : public rclcpp::Node
{
public:
  Observer()
  : rclcpp::Node("observer")
  {
    declare_parameter<std::string>("robot", "r3");
    declare_parameter<std::string>("carrier", "r1");
    declare_parameter<std::string>("dir", ".");
    // Region names, and their boxes flattened, four numbers per name.
    declare_parameter<std::vector<std::string>>("names", std::vector<std::string>{});
    declare_parameter<std::vector<double>>("boxes", std::vector<double>{});
    declare_parameter<int>("min_hits", 1);
    declare_parameter<int>("scans", 3);

    robot_ = get_parameter("robot").as_string();
    dir_ = get_parameter("dir").as_string();
    names_ = get_parameter("names").as_string_array();
    boxes_ = get_parameter("boxes").as_double_array();
    for (const auto & n : names_) {run_[n] = 0; seen_[n] = false;}

    odom_ = create_subscription<nav_msgs::msg::Odometry>(
      "/" + robot_ + "/odom", 10, [this](const nav_msgs::msg::Odometry::SharedPtr m) {
        std::lock_guard<std::mutex> lock(mutex_);
        px_ = m->pose.pose.position.x;
        py_ = m->pose.pose.position.y;
        const auto & q = m->pose.pose.orientation;
        pyaw_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
        have_pose_ = true;
      });
    scan_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/" + robot_ + "/scan", rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::LaserScan::SharedPtr m) {on_scan(m);});
    lamp_ = create_subscription<std_msgs::msg::String>(
      "/" + get_parameter("carrier").as_string() + "/lamp", 10,
      [this](const std_msgs::msg::String::SharedPtr m) {on_lamp(m->data);});
    csv_.open(dir_ + "/observer_" + robot_ + ".csv");
    csv_ << "t_sim,event,region,hits\n";
    write();
  }

private:
  bool in_box(std::size_t k, double x, double y) const
  {
    return x >= boxes_[4 * k] && y >= boxes_[4 * k + 1] && x <= boxes_[4 * k + 2] && y <= boxes_[4 * k + 3];
  }

  void on_scan(const sensor_msgs::msg::LaserScan::SharedPtr m)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!have_pose_ || boxes_.size() != 4 * names_.size()) {return;}
    const double lx = px_ - 0.032 * std::cos(pyaw_), ly = py_ - 0.032 * std::sin(pyaw_);
    std::vector<int> hits(names_.size(), 0);
    for (std::size_t i = 0; i < m->ranges.size(); ++i) {
      const double r = m->ranges[i];
      if (!std::isfinite(r) || r < m->range_min || r > m->range_max) {continue;}
      const double a = pyaw_ + m->angle_min + i * m->angle_increment;
      const double x = lx + r * std::cos(a), y = ly + r * std::sin(a);
      for (std::size_t k = 0; k < names_.size(); ++k) {hits[k] += in_box(k, x, y);}
    }
    const double t = now().seconds();
    const int need = get_parameter("min_hits").as_int();
    const int scans = get_parameter("scans").as_int();
    bool changed = false;
    for (std::size_t k = 0; k < names_.size(); ++k) {
      const auto & n = names_[k];
      run_[n] = hits[k] >= need ? run_[n] + 1 : 0;
      const bool now_seen = run_[n] >= scans;
      if (now_seen != seen_[n]) {
        seen_[n] = now_seen;
        csv_ << std::fixed << t << (now_seen ? ",begin," : ",end,") << n << "," << hits[k] << "\n";
        csv_.flush();
        if (now_seen) {
          ++sightings_[n];
          if (!first_.count(n)) {first_[n] = t;}
        }
        changed = true;
      }
    }
    if (changed) {write();}
  }

  void on_lamp(const std::string & bay)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (bay.empty()) {return;}
    // The lamp is seen only while the carrier is seen at a window.
    bool at_window = false;
    for (const auto & kv : seen_) {
      if (kv.first.rfind("window_", 0) == 0 && kv.second) {at_window = true;}
    }
    if (!at_window || lamp_seen_.count(bay)) {return;}
    lamp_seen_[bay] = now().seconds();
    csv_ << std::fixed << lamp_seen_[bay] << ",lamp," << bay << ",0\n";
    csv_.flush();
    write();
  }

  // The bay the robot can name from what it saw, or nothing.
  std::string conclusion() const
  {
    std::string got;
    auto add = [&got](const std::string & b) {
        if (got.empty()) {got = b;} else if (got != b) {got = "both";}
      };
    for (const auto & kv : lamp_seen_) {add(kv.first);}
    for (const auto & kv : first_) {
      if (kv.first == "gate_x" || kv.first == "bay_x") {add("x");}
      if (kv.first == "gate_y" || kv.first == "bay_y") {add("y");}
    }
    return got;
  }

  void write()
  {
    std::ostringstream o;
    o << "{\"robot\": \"" << robot_ << "\", \"first_seen\": {";
    bool comma = false;
    for (const auto & kv : first_) {
      o << (comma ? ", " : "") << "\"" << kv.first << "\": " << kv.second;
      comma = true;
    }
    o << "}, \"sightings\": {";
    comma = false;
    for (const auto & kv : sightings_) {
      o << (comma ? ", " : "") << "\"" << kv.first << "\": " << kv.second;
      comma = true;
    }
    o << "}, \"lamp_seen\": {";
    comma = false;
    for (const auto & kv : lamp_seen_) {
      o << (comma ? ", " : "") << "\"" << kv.first << "\": " << kv.second;
      comma = true;
    }
    o << "}, \"conclusion\": \"" << conclusion() << "\"}\n";
    const std::string path = dir_ + "/observer_" + robot_ + ".json";
    std::ofstream f(path + ".tmp");
    f << o.str();
    f.close();
    std::rename((path + ".tmp").c_str(), path.c_str());
  }

  std::string robot_, dir_;
  std::vector<std::string> names_;
  std::vector<double> boxes_;
  std::map<std::string, int> run_, sightings_;
  std::map<std::string, bool> seen_;
  std::map<std::string, double> first_, lamp_seen_;
  std::ofstream csv_;
  std::mutex mutex_;
  bool have_pose_{false};
  double px_{0.0}, py_{0.0}, pyaw_{0.0};
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr lamp_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Observer>());
  rclcpp::shutdown();
  return 0;
}
