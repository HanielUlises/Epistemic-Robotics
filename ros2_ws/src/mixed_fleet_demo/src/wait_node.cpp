// Copyright 2026 Haniel Ulises
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

// The actions of belief revision in a changing ward that wait on a laser or
// speak: look and tell. One node per action and robot; the door comes with the
// action's arguments.
//
//   look  (look r lobby_<door> view_<door><side>): the robot stands at a bend of the door's lobby,
//         s for the south leg and n for the north, and waits until its own laser
//         decides the door, on /<robot>/door (door_node.py), for at least
//         kMinLook and at most `wait` seconds; reported as
//         e-look-<door>-<side>-open or e-look-<door>-<side>-shut, the names of
//         the domain's events.
//   tell  (tell r lobby_<door>): the robot says on /radio that it found the door shut,
//         and the report is the event e-tell-<door>. It FAILS if its own laser
//         did not decide the door shut.
//   round (round p): the porter's walk at two. Asks scripts/porter_node.py for
//         it on /porter/go and waits for "round2" on /porter/done.

#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <utility>

#include "lifecycle_msgs/msg/transition.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "plansys2_executor/ActionExecutorClient.hpp"

namespace
{
constexpr double kMinLook = 3.0;    // s: a look is seen to be one
constexpr double kTell = 2.5;       // s: what a report takes to say
}  // namespace

class WaitAction : public plansys2::ActionExecutorClient
{
public:
  WaitAction()
  : plansys2::ActionExecutorClient("wait_action")
  {
    declare_parameter<std::string>("robot", "c");
    declare_parameter<std::string>("kind", "look");
    declare_parameter<double>("wait", 30.0);

    const std::string robot = get_parameter("robot").as_string();
    own_ = create_subscription<std_msgs::msg::String>(
      "/" + robot + "/door", 50,
      [this](const std_msgs::msg::String::SharedPtr m) {
        // "<door>:<side>:<open|shut>"
        const auto a = m->data.find(':');
        const auto b = m->data.rfind(':');
        if (a == std::string::npos || b == a) {return;}
        const std::string key = m->data.substr(0, a) + m->data.substr(a + 1, b - a - 1);
        std::lock_guard<std::mutex> lock(mutex_);
        decided_[key] = {m->data.substr(b + 1), now().seconds()};
      });
    radio_ = create_publisher<std_msgs::msg::String>("/radio", rclcpp::QoS(50).transient_local());
    go_ = create_publisher<std_msgs::msg::String>("/porter/go", rclcpp::QoS(10).reliable().transient_local());
    porter_done_ = create_subscription<std_msgs::msg::String>(
      "/porter/done", rclcpp::QoS(10).reliable().transient_local(),
      [this](const std_msgs::msg::String::SharedPtr m) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (m->data == "round2") {round_done_ = true;}
      });
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
    const auto & args = get_arguments();
    if (kind == "round") {
      if (started_ < 0.0) {
        started_ = now().seconds();
        std_msgs::msg::String m;
        m.data = "round2";
        go_->publish(m);
        RCLCPP_INFO(get_logger(), "round: the porter walks at two");
      }
      bool done_walking = false;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        done_walking = round_done_;
      }
      if (done_walking) {
        RCLCPP_INFO(get_logger(), "round: the porter is back, after %.1fs", now().seconds() - started_);
        done(true, "walked");
        return;
      }
      send_feedback(0.5f, "the porter is walking");
      return;
    }
    if (args.size() < 2) {
      done(false, "no door named");
      return;
    }
    if (kind == "believe") {
      // (believe r h lobby_<door> <shut|open>): a belief told to robot h, on the radio
      if (args.size() < 4) {
        done(false, "a belief report names a listener, a lobby and a verdict");
        return;
      }
      const std::string door = args[2].substr(args[2].size() - 1);
      const std::string & verdict = args[3];
      const std::string & listener = args[1];
      if (started_ < 0.0) {
        started_ = now().seconds();
        RCLCPP_INFO(get_logger(), "believe %s %s: to %s", door.c_str(), verdict.c_str(), listener.c_str());
      }
      const double said = now().seconds() - started_;
      if (said < kTell) {
        send_feedback(static_cast<float>(said / kTell), "speaking");
        return;
      }
      std_msgs::msg::String m;
      m.data = robot + ":" + door + "~" + verdict + ">" + listener;
      radio_->publish(m);
      RCLCPP_INFO(get_logger(), "believe %s %s: says %s", door.c_str(), verdict.c_str(), m.data.c_str());
      done(true, "said", "e-believe-" + door + "-" + verdict);
      return;
    }
    // look names a view, view_<door><side>; tell names a lobby, lobby_<door>
    const std::string & named = args.back();
    const std::string what = kind == "look" ? named.substr(named.size() - 2) : named.substr(named.size() - 1);
    const double t = now().seconds();
    if (started_ < 0.0) {
      started_ = t;
      RCLCPP_INFO(get_logger(), "%s %s %s: started", kind.c_str(), what.c_str(), robot.c_str());
    }
    const double waited = t - started_;

    if (kind == "tell") {
      if (waited < kTell) {
        send_feedback(static_cast<float>(waited / kTell), "speaking");
        return;
      }
      bool shut = false;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        for (const auto * side : {"s", "n"}) {
          const auto it = decided_.find(what + side);
          if (it != decided_.end() && it->second.first == "shut") {shut = true;}
        }
      }
      if (!shut) {
        RCLCPP_ERROR(get_logger(), "tell %s %s: its laser never decided the door shut", what.c_str(),
          robot.c_str());
        done(false, "not seen shut");
        return;
      }
      std_msgs::msg::String m;
      m.data = robot + ":" + what + "=shut";
      radio_->publish(m);
      RCLCPP_INFO(get_logger(), "tell %s %s: says %s", what.c_str(), robot.c_str(), m.data.c_str());
      done(true, "shut", "e-tell-" + what);
      return;
    }

    // look: a decision of this door from this side, made since the look began
    std::string decided;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const auto it = decided_.find(what);
      if (it != decided_.end() && it->second.second >= started_) {decided = it->second.first;}
    }
    if (!decided.empty() && waited >= kMinLook) {
      RCLCPP_INFO(get_logger(), "look %s %s: the laser decides %s, after %.1fs", what.c_str(), robot.c_str(),
        decided.c_str(), waited);
      done(true, decided, "e-look-" + what.substr(0, 1) + "-" + what.substr(1) + "-" + decided);
      return;
    }
    if (waited >= get_parameter("wait").as_double()) {
      RCLCPP_WARN(get_logger(), "look %s %s: the laser decides nothing in %.0fs", what.c_str(), robot.c_str(),
        waited);
      done(false, "undecided");
      return;
    }
    send_feedback(static_cast<float>(waited / get_parameter("wait").as_double()), "looking");
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr own_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr radio_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr go_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr porter_done_;
  bool round_done_{false};
  std::mutex mutex_;
  std::map<std::string, std::pair<std::string, double>> decided_;
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
