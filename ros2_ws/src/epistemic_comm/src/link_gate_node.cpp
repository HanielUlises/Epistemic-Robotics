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
// The fault injector: a relay that stops relaying while the link is down.
//
// It is installed alongside, not in the middle. The gate reads the real topic
// the producer already publishes and writes a second name that only the
// consumers of the gated copy subscribe to, so nothing that exists has to be
// remapped, rebuilt or told that a gate is present. `warehouse_demo` is
// included by the scenario launch unmodified.
//
// Interposing properly -- renaming the producer's output and taking over its
// name -- also works, and is what one would do to cut a topic that existing
// nodes already read. It is not needed here and it would mean editing a
// launch file that four other demonstrations share.
//
// The relay is type-erased. rclcpp's generic subscription and publisher carry
// a serialised message and a type name, so one gate handles an odometry, an
// occupancy grid and a speech act without a case for each, and adding a fourth
// thing to cut is a line of launch configuration rather than a code change.
//
// What it does NOT do is decide when to cut. The schedule belongs to
// comm_monitor, and this node holds no timer: it subscribes to link_down and
// link_up like everything else. Two nodes with two copies of the same schedule
// would eventually disagree about when the outage was, and the measurement
// would be against whichever copy the reader happened to trust.

#include <memory>
#include <string>
#include <vector>

#include "epistemic_msgs/msg/link_event.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialized_message.hpp"

namespace
{

rclcpp::QoS event_qos()
{
  rclcpp::QoS qos(rclcpp::KeepLast(8));
  qos.transient_local().reliable();
  return qos;
}

class LinkGate : public rclcpp::Node
{
public:
  LinkGate()
  : rclcpp::Node("link_gate")
  {
    declare_parameter<std::string>("input", "");
    declare_parameter<std::string>("output", "");
    declare_parameter<std::string>("type", "");
    declare_parameter<int>("depth", 10);
    // A map is published once every few seconds and a latched reader expects
    // to find the last one, so the gate has to be able to latch too.
    declare_parameter<bool>("transient_local", false);

    const auto source = get_parameter("input").as_string();
    topic_ = get_parameter("output").as_string();
    type_ = get_parameter("type").as_string();
    if (source.empty() || topic_.empty() || type_.empty()) {
      RCLCPP_FATAL(
        get_logger(),
        "a gate needs `input`, `output` and `type`; without the type the relay "
        "cannot be built, because a generic subscription is generic in the "
        "message and not in the type name");
      throw std::runtime_error("link_gate: input, output and type are required");
    }

    const auto depth = static_cast<std::size_t>(get_parameter("depth").as_int());
    // Braced, not parenthesised: `rclcpp::QoS qos(rclcpp::KeepLast(depth))`
    // declares a function returning QoS, which then fails to have any of QoS's
    // members. The compiler calls it a vexing parse and it is.
    rclcpp::QoS qos{rclcpp::KeepLast(depth)};
    if (get_parameter("transient_local").as_bool()) {
      qos.transient_local();
    }

    out_ = create_generic_publisher(topic_, type_, qos);
    in_ = create_generic_subscription(
      source, type_, qos,
      [this](std::shared_ptr<const rclcpp::SerializedMessage> message) {
        this->relay(*message);
      });

    down_ = create_subscription<epistemic_msgs::msg::LinkEvent>(
      "/comm_monitor/link_down", event_qos(),
      [this](epistemic_msgs::msg::LinkEvent::SharedPtr) {this->close();});
    up_ = create_subscription<epistemic_msgs::msg::LinkEvent>(
      "/comm_monitor/link_up", event_qos(),
      [this](epistemic_msgs::msg::LinkEvent::SharedPtr) {this->open();});

    RCLCPP_INFO(
      get_logger(), "relaying %s -> %s (%s)",
      source.c_str(), topic_.c_str(), type_.c_str());
  }

private:
  void relay(const rclcpp::SerializedMessage & message)
  {
    if (!open_) {
      ++dropped_;
      return;
    }
    ++passed_;
    out_->publish(message);
  }

  void close()
  {
    if (!open_) {return;}
    open_ = false;
    dropped_ = 0;
    RCLCPP_WARN(get_logger(), "%s is cut", topic_.c_str());
  }

  void open()
  {
    if (open_) {return;}
    open_ = true;
    // The count is the evidence that the cut was real. A gate that reports
    // zero dropped messages was gating a topic nobody was publishing on, and
    // that is a launch error which otherwise looks exactly like a successful
    // run.
    RCLCPP_INFO(
      get_logger(), "%s is restored; %zu message(s) were dropped while it was "
      "cut, and %zu had passed before", topic_.c_str(), dropped_, passed_);
  }

  std::string topic_;
  std::string type_;
  bool open_{true};
  std::size_t dropped_{0};
  std::size_t passed_{0};

  rclcpp::GenericPublisher::SharedPtr out_;
  rclcpp::GenericSubscription::SharedPtr in_;
  rclcpp::Subscription<epistemic_msgs::msg::LinkEvent>::SharedPtr down_;
  rclcpp::Subscription<epistemic_msgs::msg::LinkEvent>::SharedPtr up_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LinkGate>());
  rclcpp::shutdown();
  return 0;
}
