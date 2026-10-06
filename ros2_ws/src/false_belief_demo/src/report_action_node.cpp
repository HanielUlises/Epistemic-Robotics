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

// report(m, p, from, to): the mover tells the picker, over the radio, that
// the crate was moved from one bay to the other.
//
// The epistemic action is a report: the mover observes it fully, and the
// picker takes it as news of the move, relating the report to the move's
// happening in the worlds it believes. Its precondition includes the mover
// believing that the picker believes the crate is still where it was; the
// executor checks that against the model before this node is dispatched.
//
// The radio here is reliable and the report is acknowledged: the node puts
// the message on the picker's inbox, waits for it to arrive, and reports
// delivery, with the camera on the picker at its dock, where the report
// arrives. Nothing about the report is lost, which is a property of this
// floor and not of radios in general; the coordinated attack is about the
// other kind.

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "plansys2_executor/ActionExecutorClient.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class ReportAction : public plansys2::ActionExecutorClient
{
public:
  explicit ReportAction(rclcpp::Node::SharedPtr side)
  : plansys2::ActionExecutorClient("report"), side_(side)
  {
    transfer_ = side_->declare_parameter<double>("transfer_seconds", 4.0);
    radio_pub_ = side_->create_publisher<std_msgs::msg::String>(
      "/false_belief/radio", rclcpp::QoS(10).transient_local().reliable());
    inbox_pub_ = side_->create_publisher<std_msgs::msg::String>(
      "/false_belief/inbox", rclcpp::QoS(10).transient_local().reliable());
    shot_pub_ = side_->create_publisher<std_msgs::msg::String>(
      "/false_belief/shot", rclcpp::QoS(10).transient_local());
  }

private:
  void radio(const std::string & state, const std::vector<std::string> & a)
  {
    nlohmann::json msg = {{"from", a[0]}, {"to", a[1]}, {"moved_from", a[2]}, {"moved_to", a[3]},
      {"state", state}};
    std_msgs::msg::String out;
    out.data = msg.dump();
    radio_pub_->publish(out);
  }

  void do_work() override
  {
    const auto & args = get_arguments();
    if (args.size() < 4) {
      finish(false, 0.0, "report needs (report <robot> <robot> <bay> <bay>)");
      return;
    }
    if (!sending_) {
      sending_ = true;
      since_ = now();
      radio("sending", args);
      std_msgs::msg::String shot;
      shot.data = "dock";
      shot_pub_->publish(shot);
      RCLCPP_INFO(
        get_logger(), "[report] %s reports to %s: \"the crate was moved from %s to %s\"",
        args[0].c_str(), args[1].c_str(), args[2].c_str(), args[3].c_str());
    }
    if ((now() - since_).seconds() < transfer_) {
      send_feedback(0.5f, "on the air");
      return;
    }
    std_msgs::msg::String m;
    m.data = args[2] + " " + args[3];
    inbox_pub_->publish(m);
    radio("delivered", args);
    RCLCPP_INFO(
      get_logger(), "[report] delivered to %s, and acknowledged", args[1].c_str());
    sending_ = false;
    finish(true, 1.0, "reported");
  }

  rclcpp::Node::SharedPtr side_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr radio_pub_, inbox_pub_, shot_pub_;
  double transfer_{4.0};
  bool sending_{false};
  rclcpp::Time since_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto side = std::make_shared<rclcpp::Node>("report_radio");
  auto performer = std::make_shared<ReportAction>(side);
  performer->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(side);
  executor.add_node(performer->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
