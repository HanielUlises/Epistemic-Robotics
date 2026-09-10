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
// Drives the partner on a fixed patrol, so that the measurement has something
// to measure.
//
// RF-05 is a claim about a belief held while a partner is moving and nothing
// is arriving from it. Measuring it needs the partner to move, and it does not
// need the partner to be pursuing the mission: what the bound is tested
// against is a trajectory, and a trajectory that repeats is a better test
// subject than one the planner improvises, because two runs of it are
// comparable.
//
// So this is not the mission driving. It is published straight to the robot's
// velocity topic. The laser sweeps as the robot goes and SLAM builds a real map
// from it, which is what the reconciliation at the far end needs; nothing about
// the map is synthetic merely because the motion is scripted.
//
// It watches the laser. The first version drove a fixed square open loop and
// drove into a rack: a scripted trajectory is repeatable and a warehouse is
// not empty, and open loop on a floor with shelving in it is a collision with
// a timer on it. So the square is the intent and the scan is the veto. When
// the forward arc reads closer than the clearance the robot turns instead of
// advancing, and it keeps turning until the way ahead is clear. The result is
// no longer exactly a square, which costs nothing: what the measurement needs
// is a partner that moves and does not stop, not a partner that traces a
// particular figure.
//
// The patrol keeps running through the outage and knows nothing about it. A
// partner that stopped when the link fell would make the propagation right for
// a reason that has nothing to do with the propagation.

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace
{

class Patrol : public rclcpp::Node
{
public:
  Patrol()
  : rclcpp::Node("patrol")
  {
    declare_parameter<std::string>("cmd_vel", "/r1/cmd_vel");
    declare_parameter<double>("speed", 0.15);
    declare_parameter<double>("turn_rate", 0.5);
    declare_parameter<double>("side_seconds", 12.0);
    declare_parameter<double>("start_after", 5.0);
    declare_parameter<std::string>("scan", "/r1/scan");
    declare_parameter<double>("clearance", 0.55);
    declare_parameter<double>("arc", 0.6);

    speed_ = get_parameter("speed").as_double();
    turn_ = get_parameter("turn_rate").as_double();
    side_ = get_parameter("side_seconds").as_double();
    start_after_ = get_parameter("start_after").as_double();
    // A quarter turn at the configured rate, so the square closes.
    turn_seconds_ = (M_PI / 2.0) / std::max(1e-3, turn_);
    clearance_ = get_parameter("clearance").as_double();
    arc_ = get_parameter("arc").as_double();

    scan_ = create_subscription<sensor_msgs::msg::LaserScan>(
      get_parameter("scan").as_string(), rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::LaserScan::SharedPtr message) {
        this->on_scan(*message);
      });

    publisher_ = create_publisher<geometry_msgs::msg::Twist>(
      get_parameter("cmd_vel").as_string(), rclcpp::QoS(rclcpp::KeepLast(1)));
    timer_ = create_wall_timer(
      std::chrono::milliseconds(50), [this]() {this->tick();});

    RCLCPP_INFO(
      get_logger(),
      "patrolling a square of %.0f s a side at %.2f m/s, turning at %.2f rad/s",
      side_, speed_, turn_);
  }

private:
  void tick()
  {
    const auto now = this->now();
    if (begun_.nanoseconds() == 0) {
      if (now.seconds() <= 0.0) {return;}
      begun_ = now;
      return;
    }
    const double t = (now - begun_).seconds();
    if (t < start_after_) {return;}

    // Where in the square: a straight leg, then a quarter turn, repeating.
    const double cycle = side_ + turn_seconds_;
    const double phase = std::fmod(t - start_after_, cycle);

    geometry_msgs::msg::Twist command;
    const bool wants_to_advance = phase < side_;

    if (wants_to_advance && !blocked_) {
      command.linear.x = speed_;
    } else {
      // Turning, either because the square says so or because the way ahead
      // is not clear. Always the same direction: alternating would let the
      // robot oscillate in a corner without ever leaving it.
      command.angular.z = turn_;
    }
    publisher_->publish(command);
  }

  /// The nearest return in the forward arc, against the clearance.
  ///
  /// Hysteresis on the way out: once blocked, the robot needs a quarter more
  /// than the clearance before it advances again. Without it a robot that
  /// clears an obstacle by a millimetre advances, is blocked again on the next
  /// scan, and shuffles along the face of the rack it is trying to leave.
  void on_scan(const sensor_msgs::msg::LaserScan & scan)
  {
    double nearest = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
      const double angle = scan.angle_min + i * scan.angle_increment;
      if (std::abs(std::atan2(std::sin(angle), std::cos(angle))) > arc_) {
        continue;
      }
      const double r = scan.ranges[i];
      if (std::isfinite(r) && r >= scan.range_min && r <= scan.range_max) {
        nearest = std::min(nearest, static_cast<double>(r));
      }
    }
    if (!std::isfinite(nearest)) {
      return;
    }

    const bool was = blocked_;
    blocked_ = blocked_ ? (nearest < clearance_ * 1.25) : (nearest < clearance_);
    if (blocked_ != was) {
      RCLCPP_INFO(
        get_logger(), blocked_ ? "%.2f m ahead: turning" : "%.2f m ahead: clear",
        nearest);
    }
  }

  double speed_{0.0};
  double turn_{0.0};
  double side_{0.0};
  double turn_seconds_{0.0};
  double start_after_{0.0};
  double clearance_{0.55};
  double arc_{0.6};
  bool blocked_{false};
  rclcpp::Time begun_{0, 0, RCL_ROS_TIME};
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Patrol>());
  rclcpp::shutdown();
  return 0;
}
