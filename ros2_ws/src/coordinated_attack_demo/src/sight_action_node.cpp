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

// sight(a, b): the two robots see each other through t2, each from its
// viewpoint.
//
// The epistemic action is an announcement that both stand at their viewpoints,
// observed fully by a robot at a viewpoint; with both there it is public, and
// their positions become common knowledge. What makes it true on the floor is
// the sight line, and the node reads it off each robot's own laser: along the
// bearing from a robot to the other, the scan must read nothing nearer than
// the other robot. A return at the other robot's distance is the other robot
// itself; a reading beyond it means the beam passed it at the height the laser
// scans. Either way nothing stands between them, which is what seeing means
// here. The node logs both readings and refuses if either robot's beam stops
// short.

#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "coordinated_attack_demo/common.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "plansys2_executor/ActionExecutorClient.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;   // NOLINT(build/namespaces)

namespace
{

struct Pose
{
  double x{0.0}, y{0.0}, yaw{0.0};
  bool valid{false};
};

double wrap(double a)
{
  while (a > M_PI) {a -= 2.0 * M_PI;}
  while (a < -M_PI) {a += 2.0 * M_PI;}
  return a;
}

}  // namespace

class SightAction : public plansys2::ActionExecutorClient
{
public:
  explicit SightAction(rclcpp::Node::SharedPtr side)
  : plansys2::ActionExecutorClient("sight"), side_(side)
  {
    const auto agents = side_->declare_parameter<std::vector<std::string>>(
      "agents", std::vector<std::string>{"south", "north"});
    const auto namespaces = side_->declare_parameter<std::vector<std::string>>(
      "namespaces", std::vector<std::string>{"r1", "r2"});
    hold_ = side_->declare_parameter<double>("hold", 4.0);
    // How far short of the other robot's centre a return may be and still be
    // the other robot: its radius, and the laser sitting behind the centre.
    margin_ = side_->declare_parameter<double>("margin", 0.45);

    for (std::size_t k = 0; k < agents.size(); ++k) {
      const auto agent = agents[k];
      odom_sub_.push_back(side_->create_subscription<nav_msgs::msg::Odometry>(
          "/" + namespaces.at(k) + "/odom", rclcpp::SensorDataQoS(),
          [this, agent](nav_msgs::msg::Odometry::SharedPtr m) {
            const auto & q = m->pose.pose.orientation;
            pose_[agent] = {m->pose.pose.position.x, m->pose.pose.position.y,
              std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)),
              true};
          }));
      scan_sub_.push_back(side_->create_subscription<sensor_msgs::msg::LaserScan>(
          "/" + namespaces.at(k) + "/scan", rclcpp::SensorDataQoS(),
          [this, agent](sensor_msgs::msg::LaserScan::SharedPtr m) {scan_[agent] = m;}));
    }
    sight_pub_ = side_->create_publisher<std_msgs::msg::String>(
      "/coordinated_attack/sight", rclcpp::QoS(1).transient_local().reliable());
    shot_pub_ = side_->create_publisher<std_msgs::msg::String>(
      "/coordinated_attack/shot", rclcpp::QoS(10).transient_local());
  }

private:
  /// The shortest range a's laser reads within a few beams of the bearing to
  /// b, and the distance between them; negative when a reading is missing.
  std::pair<double, double> look(const std::string & a, const std::string & b)
  {
    if (!pose_.count(a) || !pose_.count(b) || !scan_.count(a) || !scan_.at(a)) {
      return {-1.0, -1.0};
    }
    const auto & pa = pose_.at(a);
    const auto & pb = pose_.at(b);
    const auto & scan = *scan_.at(a);
    const double dist = std::hypot(pb.x - pa.x, pb.y - pa.y);
    const double bearing = wrap(std::atan2(pb.y - pa.y, pb.x - pa.x) - pa.yaw);
    const int centre = static_cast<int>(std::lround((bearing - scan.angle_min) / scan.angle_increment));
    const int n = static_cast<int>(scan.ranges.size());
    double nearest = scan.range_max;
    for (int k = -3; k <= 3; ++k) {
      const int i = ((centre + k) % n + n) % n;
      const float r = scan.ranges[i];
      if (std::isfinite(r) && r >= scan.range_min) {
        nearest = std::min(nearest, static_cast<double>(r));
      }
    }
    return {nearest, dist};
  }

  void do_work() override
  {
    const auto & args = get_arguments();
    if (args.size() < 2) {
      finish(false, 0.0, "sight needs (sight <robot> <robot>)");
      return;
    }
    const auto & a = args[0];
    const auto & b = args[1];

    if (!started_) {
      started_ = true;
      since_ = now();
      std_msgs::msg::String shot;
      shot.data = "sight";
      shot_pub_->publish(shot);
    }
    if ((now() - since_).seconds() < 1.0) {
      send_feedback(0.2f, "looking through t2");
      return;
    }

    const auto [ra, da] = look(a, b);
    const auto [rb, db] = look(b, a);
    if (ra < 0.0 || rb < 0.0) {
      if ((now() - since_).seconds() > 10.0) {
        started_ = false;
        RCLCPP_ERROR(get_logger(), "[sight] no pose or scan for %s and %s", a.c_str(), b.c_str());
        finish(false, 0.0, "no pose or scan");
      }
      return;
    }
    const bool a_sees = ra >= da - margin_;
    const bool b_sees = rb >= db - margin_;
    if (!reported_) {
      reported_ = true;
      RCLCPP_INFO(
        get_logger(), "[sight] %s looks at %s %.1f m away, laser reads %.1f m: %s; %s looks at "
        "%s %.1f m away, laser reads %.1f m: %s", a.c_str(), b.c_str(), da, ra,
        a_sees ? "clear" : "blocked", b.c_str(), a.c_str(), db, rb, b_sees ? "clear" : "blocked");
      nlohmann::json msg = {{"a", a}, {"b", b}, {"clear", a_sees && b_sees}};
      std_msgs::msg::String out;
      out.data = msg.dump();
      sight_pub_->publish(out);
    }
    if (!(a_sees && b_sees)) {
      started_ = false;
      reported_ = false;
      finish(false, 0.0, "the sight line is blocked");
      return;
    }
    if ((now() - since_).seconds() < hold_) {
      send_feedback(0.6f, "in sight of each other");
      return;
    }
    started_ = false;
    reported_ = false;
    RCLCPP_INFO(get_logger(), "[sight] %s and %s in sight of each other through t2", a.c_str(),
      b.c_str());
    finish(true, 1.0, "in sight of each other");
  }

  rclcpp::Node::SharedPtr side_;
  std::map<std::string, Pose> pose_;
  std::map<std::string, sensor_msgs::msg::LaserScan::SharedPtr> scan_;
  std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> odom_sub_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr> scan_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr sight_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr shot_pub_;
  double hold_{4.0};
  double margin_{0.45};
  bool started_{false};
  bool reported_{false};
  rclcpp::Time since_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto side = std::make_shared<rclcpp::Node>("sight_lasers");
  auto performer = std::make_shared<SightAction>(side);
  performer->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(side);
  executor.add_node(performer->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
