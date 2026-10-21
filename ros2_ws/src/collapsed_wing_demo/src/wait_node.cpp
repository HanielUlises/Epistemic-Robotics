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

// The actions of the collapsed wing that wait on a map or speak: inspect,
// check and tell. One instance per action and robot.
//
//   inspect  the scout stands at its vantage and waits until its own map
//            decides the doorway, on /<robot>/decision_<door> (doorway_node),
//            for at most `wait` seconds; reported as the outcome template
//            with {v} for what the map decided, e-inspect-e-blocked;
//   check    the classical inspection: the same wait, and it SUCCEEDS when the
//            map decides the doorway clear, as the plan assumed, and FAILS with
//            "blocked" when it decides rubble, so that the mission replans;
//   tell     the scout says over the radio, /radio, what it knows of a way: of
//            its own doorway, what its own map decides; of the middle passage,
//            which nobody sees, the value by elimination: clear when both
//            doorways are known blocked, from its own map or from what it heard
//            on /radio. It FAILS when it knows neither. Reported as the
//            template with {v}, e-tell-m-clear.

#include <map>
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
    declare_parameter<std::string>("robot", "s1");
    declare_parameter<std::string>("kind", "inspect");
    declare_parameter<std::string>("door", "e");
    declare_parameter<std::string>("outcome", "");
    declare_parameter<double>("wait", 30.0);

    const std::string robot = get_parameter("robot").as_string();
    for (const std::string d : {"w", "e"}) {
      own_[d] = create_subscription<std_msgs::msg::String>(
        "/" + robot + "/decision_" + d, 10,
        [this, d](const std_msgs::msg::String::SharedPtr m) {
          std::lock_guard<std::mutex> lock(mutex_);
          mine_[d] = m->data;
        });
    }
    radio_in_ = create_subscription<std_msgs::msg::String>(
      "/radio", rclcpp::QoS(50).transient_local(),
      [this](const std_msgs::msg::String::SharedPtr m) {
        // "<door>=<value>"
        const auto eq = m->data.find('=');
        if (eq == std::string::npos) {return;}
        std::lock_guard<std::mutex> lock(mutex_);
        heard_[m->data.substr(0, eq)] = m->data.substr(eq + 1);
      });
    radio_ = create_publisher<std_msgs::msg::String>("/radio", rclcpp::QoS(50).transient_local());
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

  // What this robot knows of a doorway: its own map's decision, else what it heard.
  std::string known(const std::string & d)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (mine_.count(d) && !mine_[d].empty()) {return mine_[d];}
    if (heard_.count(d)) {return heard_[d];}
    return "";
  }

  void do_work() override
  {
    const std::string kind = get_parameter("kind").as_string();
    const std::string act = get_parameter("action_name").as_string();
    const std::string robot = get_parameter("robot").as_string();
    const std::string door = get_parameter("door").as_string();
    const std::string outcome = get_parameter("outcome").as_string();
    const double wait = get_parameter("wait").as_double();
    const double t = now().seconds();
    if (started_ < 0.0) {
      started_ = t;
      RCLCPP_INFO(get_logger(), "%s %s: started", act.c_str(), robot.c_str());
    }
    const double waited = t - started_;

    if (kind == "tell") {
      std::string v;
      if (door == "m") {
        if (known("w") == "blocked" && known("e") == "blocked") {v = "clear";}
      } else {
        std::lock_guard<std::mutex> lock(mutex_);
        v = mine_.count(door) ? mine_[door] : "";
      }
      if (v.empty()) {
        RCLCPP_ERROR(get_logger(), "%s %s: knows nothing to say of %s", act.c_str(), robot.c_str(), door.c_str());
        done(false, "nothing known");
        return;
      }
      std_msgs::msg::String m;
      m.data = door + "=" + v;
      radio_->publish(m);
      RCLCPP_INFO(get_logger(), "%s %s: says %s", act.c_str(), robot.c_str(), m.data.c_str());
      done(true, v, fill(outcome, v));
      return;
    }

    // inspect, check
    std::string decided;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      decided = mine_.count(door) ? mine_[door] : "";
    }
    if (!decided.empty()) {
      RCLCPP_INFO(get_logger(), "%s %s: the map decides %s %s, after %.1fs", act.c_str(), robot.c_str(),
        door.c_str(), decided.c_str(), waited);
      if (kind == "check" && decided == "blocked") {
        done(false, "blocked");
      } else {
        done(true, decided, fill(outcome, decided));
      }
      return;
    }
    if (waited >= wait) {
      RCLCPP_WARN(get_logger(), "%s %s: the map decides nothing in %.0fs", act.c_str(), robot.c_str(), waited);
      done(false, "undecided");
      return;
    }
    send_feedback(static_cast<float>(waited / wait), "looking");
  }

  std::map<std::string, rclcpp::Subscription<std_msgs::msg::String>::SharedPtr> own_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr radio_in_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr radio_;
  std::mutex mutex_;
  std::map<std::string, std::string> mine_, heard_;
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
