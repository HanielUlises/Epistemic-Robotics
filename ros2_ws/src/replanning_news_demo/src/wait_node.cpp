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

// The actions of replanning on news that wait on a laser or speak: look and
// tell. One instance per action and robot.
//
//   look  the robot stands at the mouth of a passage, or at the hatch, and
//         waits until its own laser decides the door, on /<robot>/door_<d>
//         (door_node.py), for at least kMinLook and at most `wait` seconds;
//         reported as e-look-<d>-open or e-look-<d>-shut;
//   tell  the robot says over the radio, /radio, whether the door is open:
//         the scout what its own laser decided, the cleaner what it saw on
//         its round (the `memory` parameter, which the launch reads from the
//         run's world). It FAILS when it knows nothing. Reported as
//         e-tell-<d>-open or e-tell-<d>-shut.

#include <map>
#include <memory>
#include <mutex>
#include <string>

#include "lifecycle_msgs/msg/transition.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "plansys2_executor/ActionExecutorClient.hpp"

namespace
{
constexpr double kMinLook = 3.0;    // s: a look is seen to be one
constexpr double kTell = 2.5;       // s: what a message takes to say
}  // namespace

class WaitAction : public plansys2::ActionExecutorClient
{
public:
  WaitAction()
  : plansys2::ActionExecutorClient("wait_action")
  {
    declare_parameter<std::string>("robot", "s");
    declare_parameter<std::string>("kind", "look");
    declare_parameter<std::string>("door", "m");
    declare_parameter<std::string>("memory", "");
    declare_parameter<double>("wait", 30.0);

    const std::string robot = get_parameter("robot").as_string();
    const std::string door = get_parameter("door").as_string();
    own_ = create_subscription<std_msgs::msg::String>(
      "/" + robot + "/door_" + door, 10,
      [this](const std_msgs::msg::String::SharedPtr m) {
        std::lock_guard<std::mutex> lock(mutex_);
        decided_ = m->data;
        decided_at_ = now().seconds();
      });
    radio_ = create_publisher<std_msgs::msg::String>("/radio", rclcpp::QoS(50).transient_local());
  }

  plansys2::CallbackReturnT on_deactivate(const rclcpp_lifecycle::State & state) override
  {
    started_ = -1.0;
    return plansys2::ActionExecutorClient::on_deactivate(state);
  }

private:
  void done(bool ok, const std::string & why, const std::string & outcome = "")
  {
    started_ = -1.0;
    finish(ok, 1.0, why, outcome);
  }

  void do_work() override
  {
    const std::string kind = get_parameter("kind").as_string();
    const std::string robot = get_parameter("robot").as_string();
    const std::string door = get_parameter("door").as_string();
    const double t = now().seconds();
    if (started_ < 0.0) {
      started_ = t;
      RCLCPP_INFO(get_logger(), "%s %s %s: started", kind.c_str(), door.c_str(), robot.c_str());
    }
    const double waited = t - started_;

    if (kind == "tell") {
      if (waited < kTell) {
        send_feedback(static_cast<float>(waited / kTell), "speaking");
        return;
      }
      std::string v = get_parameter("memory").as_string();
      if (v.empty()) {
        std::lock_guard<std::mutex> lock(mutex_);
        v = decided_;
      }
      if (v.empty()) {
        RCLCPP_ERROR(get_logger(), "tell %s %s: knows nothing to say", door.c_str(), robot.c_str());
        done(false, "nothing known");
        return;
      }
      std_msgs::msg::String m;
      m.data = robot + ":" + door + "=" + v;
      radio_->publish(m);
      RCLCPP_INFO(get_logger(), "tell %s %s: says %s", door.c_str(), robot.c_str(), m.data.c_str());
      done(true, v, "e-tell-" + door + "-" + v);
      return;
    }

    // look: a decision made since the look began
    std::string decided;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (decided_at_ >= started_) {decided = decided_;}
    }
    if (!decided.empty() && waited >= kMinLook) {
      RCLCPP_INFO(get_logger(), "look %s %s: the laser decides %s, after %.1fs", door.c_str(), robot.c_str(),
        decided.c_str(), waited);
      done(true, decided, "e-look-" + door + "-" + decided);
      return;
    }
    if (waited >= get_parameter("wait").as_double()) {
      RCLCPP_WARN(get_logger(), "look %s %s: the laser decides nothing in %.0fs", door.c_str(), robot.c_str(),
        waited);
      done(false, "undecided");
      return;
    }
    send_feedback(static_cast<float>(waited / get_parameter("wait").as_double()), "looking");
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr own_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr radio_;
  std::mutex mutex_;
  std::string decided_;
  double decided_at_{-1.0};
  double started_{-1.0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WaitAction>();
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
