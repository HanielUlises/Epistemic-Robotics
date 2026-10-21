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

// The actions of the link grades that wait on a map: inspect, share, report.
//
// One instance per action and robot:
//
//   inspect  the robot stands at its spot and waits until its own map decides
//            the corridor, on /<robot>/decision (mapper_node), for at most
//            `wait` seconds; reported as the outcome template with {v} for
//            what the map decided, e-inspect-blocked or e-inspect-clear;
//   message  the robot sends what its map decides to the other on
//            /<robot>/report, and waits for the acknowledgement: the other's
//            decision, which reaches it on /<robot>/peer_decision only through
//            the link's gate. With `ack` set it waits at most `wait` seconds;
//            with the acknowledgement it succeeds, reported as the template
//            with {v} for the value sent, e-share-blocked; without it, it
//            FAILS, and the mission replans. Without `ack` it succeeds as soon
//            as it has sent: fire and forget.
//
// The epistemic share and the classical report with replanning are both
// `message` with `ack`, with the same wait: whatever separates them in time is
// not this node.

#include <memory>
#include <mutex>
#include <string>

#include "lifecycle_msgs/msg/transition.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "plansys2_executor/ActionExecutorClient.hpp"

class WaitAction : public plansys2::ActionExecutorClient
{
public:
  WaitAction()
  : plansys2::ActionExecutorClient("wait_action")
  {
    declare_parameter<std::string>("robot", "r1");
    declare_parameter<std::string>("kind", "inspect");
    declare_parameter<std::string>("outcome", "");
    declare_parameter<double>("wait", 30.0);
    declare_parameter<bool>("ack", true);

    const std::string robot = get_parameter("robot").as_string();
    const bool inspect = get_parameter("kind").as_string() == "inspect";
    watched_ = create_subscription<std_msgs::msg::String>(
      "/" + robot + (inspect ? "/decision" : "/peer_decision"), 10,
      [this](const std_msgs::msg::String::SharedPtr m) {
        std::lock_guard<std::mutex> lock(mutex_);
        heard_ = m->data;
      });
    own_ = create_subscription<std_msgs::msg::String>(
      "/" + robot + "/decision", 10,
      [this](const std_msgs::msg::String::SharedPtr m) {
        std::lock_guard<std::mutex> lock(mutex_);
        mine_ = m->data;
      });
    report_ = create_publisher<std_msgs::msg::String>("/" + robot + "/report", 10);
  }

private:
  static std::string fill(std::string s, const std::string & value)
  {
    const auto at = s.find("{v}");
    if (at != std::string::npos) {s.replace(at, 3, value);}
    return s;
  }

  void done(bool ok, const std::string & why, const std::string & outcome = "")
  {
    started_ = -1.0;
    finish(ok, 1.0, why, outcome);
  }

  void do_work() override
  {
    const std::string kind = get_parameter("kind").as_string();
    const std::string act = get_parameter("action_name").as_string();
    const std::string robot = get_parameter("robot").as_string();
    const std::string outcome = get_parameter("outcome").as_string();
    const double wait = get_parameter("wait").as_double();
    const double t = now().seconds();
    std::string heard, mine;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      heard = heard_;
      mine = mine_;
    }
    if (started_ < 0.0) {
      started_ = t;
      sent_ = "";
      RCLCPP_INFO(get_logger(), "%s %s: started", act.c_str(), robot.c_str());
    }
    const double waited = t - started_;

    if (kind == "inspect") {
      if (!heard.empty()) {
        RCLCPP_INFO(get_logger(), "%s %s: the map decides %s, after %.1fs", act.c_str(), robot.c_str(),
          heard.c_str(), waited);
        done(true, heard, fill(outcome, heard));
        return;
      }
      if (waited >= wait) {
        RCLCPP_WARN(get_logger(), "%s %s: the map decides nothing in %.0fs", act.c_str(), robot.c_str(), waited);
        done(false, "undecided");
        return;
      }
      send_feedback(static_cast<float>(waited / wait), "looking");
      return;
    }

    // message
    if (sent_.empty()) {
      if (mine.empty()) {
        RCLCPP_ERROR(get_logger(), "%s %s: nothing to send, the map decides nothing", act.c_str(), robot.c_str());
        done(false, "nothing to send");
        return;
      }
      sent_ = mine;
    }
    std_msgs::msg::String m;
    m.data = sent_;
    report_->publish(m);
    if (!get_parameter("ack").as_bool()) {
      RCLCPP_INFO(get_logger(), "%s %s: sent %s, not waiting", act.c_str(), robot.c_str(), sent_.c_str());
      done(true, "sent", fill(outcome, sent_));
      return;
    }
    if (!heard.empty()) {
      RCLCPP_INFO(get_logger(), "%s %s: sent %s, acknowledged after %.1fs", act.c_str(), robot.c_str(),
        sent_.c_str(), waited);
      done(true, "acknowledged", fill(outcome, sent_));
      return;
    }
    if (waited >= wait) {
      RCLCPP_WARN(get_logger(), "%s %s: no acknowledgement in %.0fs", act.c_str(), robot.c_str(), waited);
      done(false, "no acknowledgement");
      return;
    }
    send_feedback(static_cast<float>(waited / wait), "waiting for the acknowledgement");
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr watched_, own_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr report_;
  std::mutex mutex_;
  std::string heard_, mine_, sent_;
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
