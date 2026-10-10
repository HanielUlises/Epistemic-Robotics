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

// A performer that does nothing itself: it hands its action to the crew and
// reports what the crew reports.
//
// The crew carries out the same four things for every fleet, and a fleet
// with no planner sends it its commands directly. So that the epistemic
// fleet's actions are carried out by the same code and not by a copy of it,
// each of its performers is this relay.
//
//     relay_action --ros-args -p action_name:=send_open
//
//   stage t            -> stage t
//   clear t            -> clear t
//   send_open i j t    -> send i j t clear     the receiver must then read t clear
//   send_blocked i j t -> send i j t blocked
//   cross h t          -> haul h t

#include <unistd.h>

#include <chrono>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "plansys2_executor/ActionExecutorClient.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;   // NOLINT(build/namespaces)
using nlohmann::json;

class Relay : public plansys2::ActionExecutorClient
{
public:
  Relay(const std::string & name, rclcpp::Node::SharedPtr side)
  : plansys2::ActionExecutorClient(name), name_(name), side_(std::move(side))
  {
    // Ids that two relays, started in two processes, cannot both use.
    next_id_ = static_cast<int>(getpid() % 10000) * 1000;
    commands_ = side_->create_publisher<std_msgs::msg::String>(
      "/stale_maps/command", rclcpp::QoS(50).reliable());
    events_ = side_->create_subscription<std_msgs::msg::String>(
      "/stale_maps/events", rclcpp::QoS(50).reliable(),
      [this](std_msgs::msg::String::SharedPtr m) {
        try {
          const auto e = json::parse(m->data);
          if (e.value("id", -1) == waiting_) {result_ = e;}
        } catch (const std::exception &) {
        }
      });
  }

private:
  bool command(json & out) const
  {
    const auto & a = get_arguments();
    if ((name_ == "stage" || name_ == "clear") && a.size() >= 1) {
      out = {{"verb", name_}, {"args", {a[0]}}};
    } else if (name_ == "send_open" && a.size() >= 3) {
      out = {{"verb", "send"}, {"args", {a[0], a[1], a[2], "clear"}}};
    } else if (name_ == "send_blocked" && a.size() >= 3) {
      out = {{"verb", "send"}, {"args", {a[0], a[1], a[2], "blocked"}}};
    } else if (name_ == "cross" && a.size() >= 2) {
      out = {{"verb", "haul"}, {"args", {a[0], a[1]}}};
    } else {
      return false;
    }
    return true;
  }

  void do_work() override
  {
    if (waiting_ < 0) {
      json c;
      if (!command(c)) {
        finish(false, 0.0, name_ + ": arguments the crew cannot take");
        return;
      }
      waiting_ = ++next_id_;
      c["id"] = waiting_;
      result_.reset();
      std_msgs::msg::String msg;
      msg.data = c.dump();
      commands_->publish(msg);
      RCLCPP_INFO(get_logger(), "[relay] %s -> crew: %s", name_.c_str(), msg.data.c_str());
      return;
    }
    if (!result_) {
      send_feedback(0.5f, "with the crew");
      return;
    }
    const bool ok = result_->value("ok", false);
    const auto message = result_->value("message", std::string{});
    waiting_ = -1;
    result_.reset();
    finish(ok, 1.0, message);
  }

  std::string name_;
  rclcpp::Node::SharedPtr side_;
  int next_id_{0};
  int waiting_{-1};
  std::optional<json> result_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr commands_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr events_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::string name = "relay";
  for (int i = 1; i + 1 < argc; ++i) {
    if (std::string(argv[i]) == "--action") {name = argv[i + 1];}
  }
  auto side = std::make_shared<rclcpp::Node>(name + "_relay_side");
  auto performer = std::make_shared<Relay>(name, side);
  performer->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(side);
  executor.add_node(performer->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
