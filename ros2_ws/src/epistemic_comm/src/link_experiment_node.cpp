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
// Measures RF-05's acceptance criterion, and writes down what it saw.
//
// It holds the two things that must not be compared carelessly: the belief,
// which is what the holder thinks while nothing is arriving, and the partner's
// actual pose, which it reads from the UNGATED topic. Reading the gated one
// would compare the propagation against the silence that produced it and would
// report an error of zero for any outage whatever.
//
// The criterion is RF-05's, verbatim: the error must not exceed
// v_max * dt + sigma_prop. This node does not decide whether the criterion is
// the right one. It reports the error, the bound, and the instants at which
// the first exceeded the second, and writes a JSON record so that a plot in
// the report is a plot of the run and not of a retelling.

#include <algorithm>
#include <cmath>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "epistemic_msgs/msg/link_event.hpp"
#include "epistemic_msgs/msg/partner_belief.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{

rclcpp::QoS event_qos()
{
  rclcpp::QoS qos(rclcpp::KeepLast(8));
  qos.transient_local().reliable();
  return qos;
}

struct Sample
{
  double t{0.0};          ///< seconds since the link fell
  double error{0.0};      ///< metres between belief and truth
  double bound{0.0};      ///< v_max*t + sigma_prop
  double trace{0.0};      ///< positional covariance trace, m^2
  double bx{0.0}, by{0.0};
  double tx{0.0}, ty{0.0};
};

class LinkExperiment : public rclcpp::Node
{
public:
  LinkExperiment()
  : rclcpp::Node("link_experiment")
  {
    declare_parameter<std::string>("belief_topic", "/r1/belief/partner");
    declare_parameter<std::string>("truth_topic", "/r2/odom/ungated");
    declare_parameter<std::string>("out", "/tmp/link_experiment.json");

    out_ = get_parameter("out").as_string();

    belief_ = create_subscription<epistemic_msgs::msg::PartnerBelief>(
      get_parameter("belief_topic").as_string(),
      rclcpp::QoS(rclcpp::KeepLast(20)),
      [this](epistemic_msgs::msg::PartnerBelief::SharedPtr message) {
        this->on_belief(*message);
      });

    truth_ = create_subscription<nav_msgs::msg::Odometry>(
      get_parameter("truth_topic").as_string(), rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::SharedPtr message) {
        truth_x_ = message->pose.pose.position.x;
        truth_y_ = message->pose.pose.position.y;
        have_truth_ = true;
      });

    down_ = create_subscription<epistemic_msgs::msg::LinkEvent>(
      "/comm_monitor/link_down", event_qos(),
      [this](epistemic_msgs::msg::LinkEvent::SharedPtr) {
        RCLCPP_INFO(get_logger(), "measuring from here");
      });
    up_ = create_subscription<epistemic_msgs::msg::LinkEvent>(
      "/comm_monitor/link_up", event_qos(),
      [this](epistemic_msgs::msg::LinkEvent::SharedPtr) {this->report();});
  }

  ~LinkExperiment() override
  {
    if (!written_) {report();}
  }

private:
  void on_belief(const epistemic_msgs::msg::PartnerBelief & message)
  {
    if (!message.propagated || !have_truth_) {return;}
    Sample s;
    s.t = message.elapsed_since_link_down;
    s.bx = message.pose.pose.position.x;
    s.by = message.pose.pose.position.y;
    s.tx = truth_x_;
    s.ty = truth_y_;
    s.error = std::hypot(s.bx - s.tx, s.by - s.ty);
    s.bound = message.admissible_error;
    s.trace = message.covariance_trace;
    samples_.push_back(s);
  }

  void report()
  {
    if (written_) {return;}
    written_ = true;

    if (samples_.empty()) {
      RCLCPP_ERROR(
        get_logger(),
        "no sample was taken: either the link never fell, or the belief node "
        "published nothing, or the truth topic carried nothing. The run "
        "measured nothing and must not be reported as a pass");
      return;
    }

    double worst = 0.0;
    double worst_at = 0.0;
    double margin = std::numeric_limits<double>::infinity();
    std::size_t breaches = 0;
    double first_breach = -1.0;
    for (const auto & s : samples_) {
      if (s.error > worst) {worst = s.error; worst_at = s.t;}
      margin = std::min(margin, s.bound - s.error);
      if (s.error > s.bound) {
        ++breaches;
        if (first_breach < 0.0) {first_breach = s.t;}
      }
    }
    const auto & last = samples_.back();

    RCLCPP_INFO(
      get_logger(),
      "%zu samples over %.1f s of outage; worst error %.3f m at t=%.1f s; "
      "tightest margin %.3f m; %zu breach(es) of the RF-05 bound",
      samples_.size(), last.t, worst, worst_at, margin, breaches);
    if (breaches == 0) {
      RCLCPP_INFO(get_logger(), "RF-05 acceptance criterion: MET");
    } else {
      RCLCPP_WARN(
        get_logger(), "RF-05 acceptance criterion: NOT met, first at t=%.1f s",
        first_breach);
    }

    std::ofstream file(out_);
    if (!file) {
      RCLCPP_ERROR(get_logger(), "cannot write %s", out_.c_str());
      return;
    }
    file.setf(std::ios::fixed);
    file.precision(4);
    file << "{\n  \"outage_seconds\": " << last.t
         << ",\n  \"samples\": " << samples_.size()
         << ",\n  \"worst_error_m\": " << worst
         << ",\n  \"worst_error_at_s\": " << worst_at
         << ",\n  \"tightest_margin_m\": " << margin
         << ",\n  \"breaches\": " << breaches
         << ",\n  \"criterion_met\": " << (breaches == 0 ? "true" : "false")
         << ",\n  \"final_trace_m2\": " << last.trace
         << ",\n  \"series\": [\n";
    for (std::size_t i = 0; i < samples_.size(); ++i) {
      const auto & s = samples_[i];
      file << "    {\"t\": " << s.t << ", \"error\": " << s.error
           << ", \"bound\": " << s.bound << ", \"trace\": " << s.trace
           << ", \"belief\": [" << s.bx << ", " << s.by
           << "], \"truth\": [" << s.tx << ", " << s.ty << "]}"
           << (i + 1 < samples_.size() ? "," : "") << "\n";
    }
    file << "  ]\n}\n";
    RCLCPP_INFO(get_logger(), "wrote %s", out_.c_str());
  }

  std::string out_;
  bool written_{false};
  bool have_truth_{false};
  double truth_x_{0.0};
  double truth_y_{0.0};
  std::vector<Sample> samples_;

  rclcpp::Subscription<epistemic_msgs::msg::PartnerBelief>::SharedPtr belief_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr truth_;
  rclcpp::Subscription<epistemic_msgs::msg::LinkEvent>::SharedPtr down_;
  rclcpp::Subscription<epistemic_msgs::msg::LinkEvent>::SharedPtr up_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LinkExperiment>());
  rclcpp::shutdown();
  return 0;
}
