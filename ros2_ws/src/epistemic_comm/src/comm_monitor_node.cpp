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
// Owns the state of the radio link, and is the only thing that does.
//
// RF-05 names /comm_monitor/link_down and /comm_monitor/link_up but is of two
// minds about what they are: the requirement's prose calls link_down a
// service, and the requirement tables of chapters 3 and 4 call it a topic.
// They are topics here, and a service answers the question a topic cannot: a
// node that starts late needs to know the state now, and asking a topic for
// the present state means waiting for the next transition.
//
// The schedule is open loop. The link falls at `t_disc` and returns at
// `t_recon`, both measured from the moment this node sees its first clock
// tick, and neither depends on where the robots are. That is deliberate: a
// disconnection triggered by distance would make the outage a function of the
// very trajectory the outage perturbs, and the measurement could not then
// separate the two.

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "epistemic_msgs/msg/link_event.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace
{

/// Link events are latched. A gate or a belief engine that comes up after the
/// link already fell must learn that it fell, and the alternative to a latched
/// topic is for every consumer to call the service at startup and to get the
/// ordering right.
rclcpp::QoS event_qos()
{
  rclcpp::QoS qos(rclcpp::KeepLast(8));
  qos.transient_local().reliable();
  return qos;
}

class CommMonitor : public rclcpp::Node
{
public:
  CommMonitor()
  : rclcpp::Node("comm_monitor")
  {
    declare_parameter<std::vector<std::string>>("pair", {"r1", "r2"});
    declare_parameter<double>("t_disc", 30.0);
    declare_parameter<double>("t_recon", 60.0);

    pair_ = get_parameter("pair").as_string_array();
    if (pair_.size() != 2) {
      RCLCPP_FATAL(
        get_logger(),
        "pair must name exactly two robots; a partition of more than two is "
        "out of scope for this scenario and would need a link per edge");
      throw std::runtime_error("comm_monitor: pair must have two entries");
    }
    t_disc_ = get_parameter("t_disc").as_double();
    t_recon_ = get_parameter("t_recon").as_double();
    if (!(t_recon_ > t_disc_)) {
      RCLCPP_FATAL(
        get_logger(), "t_recon (%.2f) must be after t_disc (%.2f)",
        t_recon_, t_disc_);
      throw std::runtime_error("comm_monitor: t_recon must follow t_disc");
    }

    down_ = create_publisher<epistemic_msgs::msg::LinkEvent>(
      "/comm_monitor/link_down", event_qos());
    up_ = create_publisher<epistemic_msgs::msg::LinkEvent>(
      "/comm_monitor/link_up", event_qos());

    state_ = create_service<std_srvs::srv::Trigger>(
      "/comm_monitor/state",
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        response->success = link_up_;
        response->message = link_up_ ? "up" : "down";
      });

    timer_ = create_wall_timer(
      std::chrono::milliseconds(50), [this]() {this->tick();});

    RCLCPP_INFO(
      get_logger(),
      "watching the link between %s and %s; it falls at t=%.1f s and returns "
      "at t=%.1f s, both from this node's first tick",
      pair_[0].c_str(), pair_[1].c_str(), t_disc_, t_recon_);
  }

private:
  void tick()
  {
    const auto now = this->now();
    if (!started_) {
      // The first tick, and not construction, starts the clock. Under
      // simulated time a node built before the clock publisher exists reads
      // zero, and a schedule anchored on that fires everything at once.
      if (now.seconds() <= 0.0) {
        return;
      }
      begun_ = now;
      started_ = true;
      RCLCPP_INFO(get_logger(), "clock started at %.2f s", begun_.seconds());
      return;
    }

    const double elapsed = (now - begun_).seconds();
    if (link_up_ && elapsed >= t_disc_) {
      publish(false, now, elapsed);
      link_up_ = false;
      changed_at_ = elapsed;
      RCLCPP_WARN(
        get_logger(), "link between %s and %s is DOWN at t=%.2f s",
        pair_[0].c_str(), pair_[1].c_str(), elapsed);
      return;
    }
    if (!link_up_ && elapsed >= t_recon_) {
      publish(true, now, elapsed);
      link_up_ = true;
      changed_at_ = elapsed;
      RCLCPP_INFO(
        get_logger(), "link between %s and %s is UP at t=%.2f s, after %.2f s "
        "of outage", pair_[0].c_str(), pair_[1].c_str(), elapsed,
        elapsed - t_disc_);
      timer_->cancel();
    }
  }

  void publish(bool up, const rclcpp::Time & stamp, double elapsed)
  {
    epistemic_msgs::msg::LinkEvent event;
    event.header.stamp = stamp;
    event.header.frame_id = pair_[0] + "|" + pair_[1];
    event.pair[0] = pair_[0];
    event.pair[1] = pair_[1];
    event.up = up;
    event.previous_state_held_for = elapsed - changed_at_;
    (up ? up_ : down_)->publish(event);
  }

  std::vector<std::string> pair_;
  double t_disc_{0.0};
  double t_recon_{0.0};

  bool started_{false};
  bool link_up_{true};
  double changed_at_{0.0};
  rclcpp::Time begun_;

  rclcpp::Publisher<epistemic_msgs::msg::LinkEvent>::SharedPtr down_;
  rclcpp::Publisher<epistemic_msgs::msg::LinkEvent>::SharedPtr up_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr state_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CommMonitor>());
  rclcpp::shutdown();
  return 0;
}
