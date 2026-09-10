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
// RF-05's BeliefUpdateEngine: one robot's belief about where its partner is.
//
// While the link is up it holds the last pose that arrived. While the link is
// down nothing arrives, and it integrates the last twist it saw forward from
// the last pose it saw, growing the covariance as it goes. That is the whole
// of it, and the modesty is the point: the requirement asks for a prediction
// under the last shared plan, and the last twist received is what a shared
// plan reduces to at the instant it stopped being shared.
//
// It subscribes to the GATED partner odometry, which is what makes the outage
// real for it. The measurement node subscribes to the ungated one, so that the
// error being measured is against where the partner actually went and not
// against the same silence this node is reasoning from.

#include <array>
#include <chrono>
#include <cstdio>
#include <memory>
#include <string>

#include "epistemic_comm/belief_propagation.hpp"
#include "epistemic_msgs/msg/link_event.hpp"
#include "epistemic_msgs/msg/partner_belief.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace
{

rclcpp::QoS event_qos()
{
  rclcpp::QoS qos(rclcpp::KeepLast(8));
  qos.transient_local().reliable();
  return qos;
}

double yaw_of(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

class BeliefUpdate : public rclcpp::Node
{
public:
  BeliefUpdate()
  : rclcpp::Node("belief_update")
  {
    declare_parameter<std::string>("holder", "r1");
    declare_parameter<std::string>("subject", "r2");
    declare_parameter<std::string>("partner_odom", "/r2/odom");
    declare_parameter<double>("rate", 10.0);
    declare_parameter<double>("v_max", 0.22);
    declare_parameter<double>("sigma_prop", 0.05);
    declare_parameter<double>("variance_rate", 0.01);
    declare_parameter<double>("yaw_variance_rate", 0.005);
    declare_parameter<double>("sigma_max_squared", 1.0);

    holder_ = get_parameter("holder").as_string();
    subject_ = get_parameter("subject").as_string();
    params_.v_max = get_parameter("v_max").as_double();
    params_.sigma_prop = get_parameter("sigma_prop").as_double();
    params_.variance_rate = get_parameter("variance_rate").as_double();
    params_.yaw_variance_rate = get_parameter("yaw_variance_rate").as_double();
    params_.sigma_max_squared = get_parameter("sigma_max_squared").as_double();

    odom_ = create_subscription<nav_msgs::msg::Odometry>(
      get_parameter("partner_odom").as_string(), rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::SharedPtr message) {
        this->on_partner(*message);
      });

    down_ = create_subscription<epistemic_msgs::msg::LinkEvent>(
      "/comm_monitor/link_down", event_qos(),
      [this](epistemic_msgs::msg::LinkEvent::SharedPtr event) {
        this->on_down(*event);
      });
    up_ = create_subscription<epistemic_msgs::msg::LinkEvent>(
      "/comm_monitor/link_up", event_qos(),
      [this](epistemic_msgs::msg::LinkEvent::SharedPtr event) {
        this->on_up(*event);
      });

    belief_ = create_publisher<epistemic_msgs::msg::PartnerBelief>(
      "/" + holder_ + "/belief/partner", rclcpp::QoS(rclcpp::KeepLast(20)));

    // The belief, drawn. A number in a log says the estimate drifted; a ghost
    // that separates from the robot and snaps back when the link returns is
    // the same fact in the form the demonstration needs.
    // Latched. A viewer that connects after the belief started publishing
    // should find the last state rather than an empty screen, and the display
    // that reads this asks for transient local: a volatile publisher against
    // it is an incompatible pair, which the middleware reports once, as a
    // line in a log, and then draws nothing for the rest of the run.
    rclcpp::QoS marker_qos{rclcpp::KeepLast(4)};
    marker_qos.transient_local().reliable();
    markers_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/" + holder_ + "/belief/markers", marker_qos);

    const auto period = std::chrono::duration<double>(
      1.0 / std::max(1.0, get_parameter("rate").as_double()));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() {this->publish();});

    RCLCPP_INFO(
      get_logger(), "%s believes about %s; bound is %.3f*dt + %.3f m",
      holder_.c_str(), subject_.c_str(), params_.v_max, params_.sigma_prop);
  }

private:
  void on_partner(const nav_msgs::msg::Odometry & message)
  {
    if (!link_up_) {
      // Nothing should arrive while the link is down, because the gate is
      // upstream. If something does, the gate is not on this topic and the
      // run is not measuring what it claims to.
      RCLCPP_ERROR_ONCE(
        get_logger(),
        "a partner pose arrived while the link was down: this node is reading "
        "an ungated topic and the outage is not being applied to it");
      return;
    }
    frame_ = message.header.frame_id;
    pose_.x = message.pose.pose.position.x;
    pose_.y = message.pose.pose.position.y;
    pose_.yaw = yaw_of(message.pose.pose.orientation);
    twist_.v = message.twist.twist.linear.x;
    twist_.w = message.twist.twist.angular.z;
    for (std::size_t i = 0; i < 36; ++i) {
      covariance_[i] = message.pose.covariance[i];
    }
    held_ = true;
    last_received_ = now();
  }

  void on_down(const epistemic_msgs::msg::LinkEvent & event)
  {
    if (!link_up_) {return;}
    link_up_ = false;
    fell_at_ = event.header.stamp;
    at_link_down_ = covariance_;
    RCLCPP_WARN(
      get_logger(),
      "link down; propagating %s from (%.2f, %.2f) under v=%.3f m/s, "
      "w=%.3f rad/s", subject_.c_str(), pose_.x, pose_.y, twist_.v, twist_.w);
  }

  void on_up(const epistemic_msgs::msg::LinkEvent &)
  {
    if (link_up_) {return;}
    link_up_ = true;
    RCLCPP_INFO(
      get_logger(),
      "link up; the belief was propagated for %.2f s and reached a trace of "
      "%.4f m^2", elapsed_, epistemic_comm::positional_trace(covariance_));
  }

  void publish()
  {
    if (!held_) {return;}

    if (!link_up_) {
      const double now_s = now().seconds();
      const double fell_s = rclcpp::Time(fell_at_).seconds();
      const double step = now_s - last_step_s_;
      elapsed_ = now_s - fell_s;
      if (last_step_s_ > 0.0 && step > 0.0) {
        pose_ = epistemic_comm::integrate(pose_, twist_, step);
      }
      last_step_s_ = now_s;
      covariance_ = epistemic_comm::propagated_covariance(
        at_link_down_, elapsed_, params_);
    } else {
      elapsed_ = 0.0;
      last_step_s_ = 0.0;
    }

    epistemic_msgs::msg::PartnerBelief message;
    message.header.stamp = now();
    message.header.frame_id = frame_.empty() ? "odom" : frame_;
    message.holder = holder_;
    message.subject = subject_;
    message.pose.pose.position.x = pose_.x;
    message.pose.pose.position.y = pose_.y;
    message.pose.pose.orientation.z = std::sin(pose_.yaw / 2.0);
    message.pose.pose.orientation.w = std::cos(pose_.yaw / 2.0);
    for (std::size_t i = 0; i < 36; ++i) {
      message.pose.covariance[i] = covariance_[i];
    }
    message.propagated = !link_up_;
    message.elapsed_since_link_down = elapsed_;
    message.admissible_error =
      epistemic_comm::admissible_error(elapsed_, params_);
    message.covariance_trace =
      epistemic_comm::positional_trace(covariance_);
    belief_->publish(message);
    draw(message);

    // RF-05's invariant: past sigma_max^2 the position of the partner is no
    // longer something this robot believes, and saying so once is the whole of
    // what this node can do about it. Marking the proposition uncertain in the
    // epistemic model is the executor's to do, and there is no per-robot model
    // to mark it in.
    if (!link_up_ && !warned_ &&
      message.covariance_trace > params_.sigma_max_squared)
    {
      warned_ = true;
      RCLCPP_WARN(
        get_logger(),
        "trace %.3f m^2 is past sigma_max^2 = %.3f after %.1f s: the estimate "
        "of where %s is should no longer be believed",
        message.covariance_trace, params_.sigma_max_squared, elapsed_,
        subject_.c_str());
    }
  }

  /// Draw the belief: where the holder thinks its partner is, and how sure it
  /// is of that. The disc is the positional covariance at one standard
  /// deviation, so it grows as the estimate ages, and it turns red once the
  /// trace passes the threshold past which RF-05 says the estimate should no
  /// longer be believed.
  void draw(const epistemic_msgs::msg::PartnerBelief & belief)
  {
    visualization_msgs::msg::MarkerArray array;
    const auto stamp = belief.header.stamp;
    const auto frame = belief.header.frame_id;
    const bool stale = belief.covariance_trace > params_.sigma_max_squared;

    visualization_msgs::msg::Marker ghost;
    ghost.header.stamp = stamp;
    ghost.header.frame_id = frame;
    ghost.ns = "belief";
    ghost.id = 0;
    ghost.type = visualization_msgs::msg::Marker::ARROW;
    ghost.action = visualization_msgs::msg::Marker::ADD;
    ghost.pose = belief.pose.pose;
    ghost.scale.x = 0.45;
    ghost.scale.y = 0.09;
    ghost.scale.z = 0.09;
    ghost.color.r = stale ? 0.78f : 0.10f;
    ghost.color.g = stale ? 0.06f : 0.10f;
    ghost.color.b = stale ? 0.18f : 0.10f;
    ghost.color.a = belief.propagated ? 0.95f : 0.35f;
    array.markers.push_back(ghost);

    visualization_msgs::msg::Marker disc = ghost;
    disc.id = 1;
    disc.type = visualization_msgs::msg::Marker::CYLINDER;
    // One standard deviation of the positional covariance, as a diameter.
    const double sigma = std::sqrt(std::max(0.0, belief.covariance_trace / 2.0));
    disc.scale.x = std::max(0.12, 2.0 * sigma);
    disc.scale.y = disc.scale.x;
    disc.scale.z = 0.01;
    disc.pose.position.z = 0.01;
    disc.color.a = belief.propagated ? 0.22f : 0.06f;
    array.markers.push_back(disc);

    visualization_msgs::msg::Marker label = ghost;
    label.id = 2;
    label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    label.pose.position.z = 0.6;
    label.scale.z = 0.28;
    label.color.a = 0.95f;
    if (!belief.propagated) {
      label.text = holder_ + " sees " + subject_;
    } else {
      char line[96];
      std::snprintf(
        line, sizeof(line), "%s believes %s  %.0f s  %.2f m2",
        holder_.c_str(), subject_.c_str(),
        belief.elapsed_since_link_down, belief.covariance_trace);
      label.text = line;
    }
    array.markers.push_back(label);

    markers_->publish(array);
  }

  std::string holder_;
  std::string subject_;
  std::string frame_;
  epistemic_comm::PropagationParams params_;

  bool held_{false};
  bool link_up_{true};
  bool warned_{false};
  double elapsed_{0.0};
  double last_step_s_{0.0};
  epistemic_comm::Pose2D pose_;
  epistemic_comm::Twist2D twist_;
  std::array<double, 36> covariance_{};
  std::array<double, 36> at_link_down_{};
  builtin_interfaces::msg::Time fell_at_;
  rclcpp::Time last_received_{0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_;
  rclcpp::Subscription<epistemic_msgs::msg::LinkEvent>::SharedPtr down_;
  rclcpp::Subscription<epistemic_msgs::msg::LinkEvent>::SharedPtr up_;
  rclcpp::Publisher<epistemic_msgs::msg::PartnerBelief>::SharedPtr belief_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BeliefUpdate>());
  rclcpp::shutdown();
  return 0;
}
