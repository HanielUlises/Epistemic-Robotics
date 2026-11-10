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
// The actions of a faulty sensor in the fleet that wait on a laser or speak.
// One node per action and robot; the door comes with the action's arguments.
//
//   look  (look r pass_<door> <view>_<door>): the robot stands at the door's
//         mouth (view far, or far1 and far2 where the domain tells the first
//         look from the second) or at the mark before it (view near) and reads
//         the door with its own laser, on /<robot>/door (door_node.py), for
//         kMinLook seconds: shut if any scan in that time decided shut, open if
//         every one decided open. Reported as e-look-<door>-<reading>,
//         e-look<k>-<door>-<reading> or e-check-<door>-<reading>, the names of
//         the domain's events.
//   say   (say r pass_<door> <word>): the robot says its reading on /radio, as
//         "<robot>:<door>=<word>", the word open or readshut (what a look from
//         the mouth read; open1, open2, readshut1, readshut2 where looks are
//         told apart), shut or sure (read at the mark); the report is the
//         domain's event of that name. It FAILS if the robot's own laser never
//         read what it says.

#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <utility>

#include "lifecycle_msgs/msg/transition.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "plansys2_executor/ActionExecutorClient.hpp"

namespace
{
constexpr double kMinLook = 4.0;    // s: the scans a reading is taken over
constexpr double kSay = 2.5;        // s: what a report takes to say
}  // namespace

class WaitAction : public plansys2::ActionExecutorClient
{
public:
  WaitAction()
  : plansys2::ActionExecutorClient("wait_action")
  {
    declare_parameter<std::string>("robot", "s");
    declare_parameter<std::string>("kind", "look");
    declare_parameter<double>("wait", 30.0);
    const std::string robot = get_parameter("robot").as_string();
    own_ = create_subscription<std_msgs::msg::String>(
      "/" + robot + "/door", 50,
      [this](const std_msgs::msg::String::SharedPtr m) {
        // "<door>:<far|near>:<open|shut>"
        const auto a = m->data.find(':');
        const auto b = m->data.rfind(':');
        if (a == std::string::npos || b == a) {return;}
        const std::string key = m->data.substr(0, b);
        const std::string verdict = m->data.substr(b + 1);
        std::lock_guard<std::mutex> lock(mutex_);
        ever_.insert(m->data);
        if (started_ >= 0.0) {
          if (verdict == "shut") {++shut_[key];} else {++open_[key];}
        }
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
    const auto & args = get_arguments();
    if (args.size() < 3) {
      done(false, "no door named");
      return;
    }
    const std::string door = args[1].substr(args[1].size() - 1);
    const double t = now().seconds();
    if (started_ < 0.0) {
      std::lock_guard<std::mutex> lock(mutex_);
      started_ = t;
      shut_.clear();
      open_.clear();
      RCLCPP_INFO(get_logger(), "%s %s %s: started", kind.c_str(), args[2].c_str(), robot.c_str());
    }
    const double waited = t - started_;
    if (kind == "say") {
      const std::string & word = args[2];
      if (waited < kSay) {
        send_feedback(static_cast<float>(waited / kSay), "speaking");
        return;
      }
      bool read = false;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (word == "shut") {
          read = ever_.count(door + ":near:shut");
        } else if (word.rfind("readshut", 0) == 0) {
          read = ever_.count(door + ":far:shut");
        } else if (word == "sure") {
          read = ever_.count(door + ":near:open");
        } else {
          read = ever_.count(door + ":far:open");
        }
      }
      if (!read) {
        RCLCPP_ERROR(get_logger(), "say %s %s %s: its laser never read that", word.c_str(), door.c_str(),
          robot.c_str());
        done(false, "not read");
        return;
      }
      std_msgs::msg::String m;
      m.data = robot + ":" + door + "=" + word;
      radio_->publish(m);
      RCLCPP_INFO(get_logger(), "say: %s", m.data.c_str());
      // the domain's event: e-say-open<k>-<door>, e-say-read-shut<k>-<door>, e-say-shut-<door>, e-say-sure-<door>
      const std::string name = word.rfind("readshut", 0) == 0 ? "read-shut" + word.substr(8) : word;
      done(true, word, "e-say-" + name + "-" + door);
      return;
    }
    // look: view far_<door>, far<k>_<door> or near_<door>
    const std::string view = args[2].substr(0, args[2].find('_'));
    const std::string where = view == "near" ? "near" : "far";
    const std::string key = door + ":" + where;
    int shut = 0, open = 0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      shut = shut_[key];
      open = open_[key];
    }
    if (waited >= kMinLook && shut + open > 0) {
      const std::string reading = shut > 0 ? "shut" : "open";
      const std::string event = view == "near" ? "e-check-" + door + "-" + reading :
        "e-look" + view.substr(3) + "-" + door + "-" + reading;
      RCLCPP_INFO(get_logger(), "look %s %s: reads %s (%d scans shut, %d open) after %.1fs", args[2].c_str(),
        robot.c_str(), reading.c_str(), shut, open, waited);
      done(true, reading, event);
      return;
    }
    if (waited >= get_parameter("wait").as_double()) {
      RCLCPP_WARN(get_logger(), "look %s %s: the laser decides nothing in %.0fs", args[2].c_str(), robot.c_str(),
        waited);
      done(false, "undecided");
      return;
    }
    send_feedback(static_cast<float>(waited / get_parameter("wait").as_double()), "looking");
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr own_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr radio_;
  std::mutex mutex_;
  std::set<std::string> ever_;
  std::map<std::string, int> shut_, open_;
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
