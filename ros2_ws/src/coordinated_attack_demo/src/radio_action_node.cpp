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

// tell, ack, ack2, ack3 (i, j, s): one radio message from i to j, at one level
// of nesting.
//
// The epistemic action is a lossy message: delivered, lost, or nothing sent,
// with the sender unable to tell the first two apart and the receiver unable
// to tell the last two apart. Only delivery is designated, and this node
// delivers: the message goes out on the receiver's inbox and the node waits
// for the receiver's radio to report it. What the sender would see of that
// report is nothing, and nothing in the node gives it to the sender; the
// report is the system's check that the designated event is the one that
// happened. A message moves nobody, so the camera is sent to a wide shot of the
// block the message crosses.
//
//     radio_action --kind ack    (one process per message level)

#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "coordinated_attack_demo/common.hpp"
#include "plansys2_executor/ActionExecutorClient.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;   // NOLINT(build/namespaces)

namespace
{

/// What the message says: the sender knows the content of the one before.
std::string content(const std::string & kind, const std::string & i, const std::string & j,
  const std::string & s)
{
  const std::string job = "job(" + s + ")";
  if (kind == "tell") {return "K_" + i + " " + job;}
  if (kind == "ack") {return "K_" + i + " K_" + j + " " + job;}
  if (kind == "ack2") {return "K_" + i + " K_" + j + " K_" + i + " " + job;}
  return "K_" + i + " K_" + j + " K_" + i + " K_" + j + " " + job;
}

}  // namespace

class RadioAction : public plansys2::ActionExecutorClient
{
public:
  RadioAction(const std::string & kind, rclcpp::Node::SharedPtr side)
  : plansys2::ActionExecutorClient("radio_" + kind), kind_(kind), side_(side)
  {
    transfer_ = side_->declare_parameter<double>("transfer_seconds", 4.0);
    const auto agents = side_->declare_parameter<std::vector<std::string>>(
      "agents", std::vector<std::string>{"south", "north"});
    for (const auto & a : agents) {
      inbox_pub_[a] = side_->create_publisher<std_msgs::msg::String>(
        "/coordinated_attack/inbox/" + a, rclcpp::QoS(10).reliable());
      receipt_sub_[a] = side_->create_subscription<std_msgs::msg::String>(
        "/coordinated_attack/inbox/" + a, rclcpp::QoS(10).reliable(),
        [this](std_msgs::msg::String::SharedPtr msg) {last_received_ = msg->data;});
    }
    radio_pub_ = side_->create_publisher<std_msgs::msg::String>(
      "/coordinated_attack/radio", rclcpp::QoS(10).transient_local());
    shot_pub_ = side_->create_publisher<std_msgs::msg::String>(
      "/coordinated_attack/shot", rclcpp::QoS(10).transient_local());
  }

private:
  void publish_radio(const std::string & state, const std::string & i, const std::string & j,
    const std::string & s)
  {
    nlohmann::json msg = {{"kind", kind_}, {"from", i}, {"to", j}, {"stand", s},
      {"content", content(kind_, i, j, s)}, {"state", state}};
    std_msgs::msg::String out;
    out.data = msg.dump();
    radio_pub_->publish(out);
  }

  void do_work() override
  {
    const auto & args = get_arguments();
    if (args.size() < 3 || !inbox_pub_.count(args[1])) {
      finish(false, 0.0, "radio needs (radio_<kind> <from> <to> <stand>)");
      return;
    }
    const auto & i = args[0];
    const auto & j = args[1];
    const auto & s = args[2];

    if (!sending_) {
      sending_ = true;
      started_ = now();
      token_ = kind_ + " " + i + " " + j + " " + s + " " + std::to_string(started_.nanoseconds());
      last_received_.clear();
      publish_radio("sending", i, j, s);
      std_msgs::msg::String shot;
      shot.data = "radio";
      shot_pub_->publish(shot);
      RCLCPP_INFO(
        get_logger(), "[radio] %s(%s, %s, %s): %s sends \"%s\" to %s", kind_.c_str(), i.c_str(),
        j.c_str(), s.c_str(), i.c_str(), content(kind_, i, j, s).c_str(), j.c_str());
    }

    const double elapsed = (now() - started_).seconds();
    if (elapsed >= 0.5 * transfer_ && !sent_) {
      sent_ = true;
      std_msgs::msg::String m;
      m.data = token_;
      inbox_pub_.at(j)->publish(m);
    }
    if (elapsed < transfer_) {
      send_feedback(static_cast<float>(elapsed / transfer_), "on the air");
      return;
    }
    sending_ = false;
    sent_ = false;
    if (last_received_ != token_) {
      publish_radio("idle", i, j, s);
      RCLCPP_ERROR(
        get_logger(), "[radio] %s(%s, %s, %s) did not reach %s; the designated event did not "
        "happen", kind_.c_str(), i.c_str(), j.c_str(), s.c_str(), j.c_str());
      finish(false, 1.0, "not delivered");
      return;
    }
    publish_radio("delivered", i, j, s);
    RCLCPP_INFO(
      get_logger(), "[radio] %s(%s, %s, %s) delivered to %s; %s cannot tell whether it was",
      kind_.c_str(), i.c_str(), j.c_str(), s.c_str(), j.c_str(), i.c_str());
    finish(true, 1.0, "delivered");
  }

  std::string kind_;
  rclcpp::Node::SharedPtr side_;
  double transfer_{4.0};
  std::map<std::string, rclcpp::Publisher<std_msgs::msg::String>::SharedPtr> inbox_pub_;
  std::map<std::string, rclcpp::Subscription<std_msgs::msg::String>::SharedPtr> receipt_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr radio_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr shot_pub_;
  std::string token_;
  std::string last_received_;
  bool sending_{false};
  bool sent_{false};
  rclcpp::Time started_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  const auto kind = coordinated_attack::argument(argc, argv, "--kind", "tell");

  auto side = std::make_shared<rclcpp::Node>("radio_" + kind + "_link");
  auto performer = std::make_shared<RadioAction>(kind, side);
  performer->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(side);
  executor.add_node(performer->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
