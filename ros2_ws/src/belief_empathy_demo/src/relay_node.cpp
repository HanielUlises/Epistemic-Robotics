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

// Every action of the belief and empathy demonstration, as the executor sees
// it. One instance per action and robot. The behaviour is the robot's own node
// (scripts/uav_node.py, scripts/ugv_node.py), which every formulation drives
// the same way: this node hands it the action, on /<robot>/request as
// {"id", "action", "args"}, and waits for the answer with the same id on
// /<robot>/reply, {"id", "ok", "outcome", "why"}. The outcome, when there is
// one, is the event that occurred, e-met-1 or e-absent-1, which the epistemic
// executor uses to choose the policy's branch. An action that has no answer in
// `timeout` seconds FAILS.

#include <memory>
#include <mutex>
#include <string>

#include "lifecycle_msgs/msg/transition.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "plansys2_executor/ActionExecutorClient.hpp"

namespace
{
// The answer's fields, from the flat JSON the robot nodes write. Values are
// strings or booleans, and no string holds a quote.
std::string field(const std::string & json, const std::string & key)
{
  const auto at = json.find("\"" + key + "\"");
  if (at == std::string::npos) {return "";}
  auto v = json.find(':', at) + 1;
  while (v < json.size() && json[v] == ' ') {++v;}
  if (v < json.size() && json[v] == '"') {
    const auto end = json.find('"', v + 1);
    return json.substr(v + 1, end - v - 1);
  }
  const auto end = json.find_first_of(",}", v);
  return json.substr(v, end - v);
}
}  // namespace

class RelayAction : public plansys2::ActionExecutorClient
{
public:
  RelayAction()
  : plansys2::ActionExecutorClient("relay_action")
  {
    declare_parameter<std::string>("robot", "u");
    declare_parameter<double>("timeout", 900.0);
    const std::string robot = get_parameter("robot").as_string();
    request_ = create_publisher<std_msgs::msg::String>("/" + robot + "/request", 10);
    reply_ = create_subscription<std_msgs::msg::String>(
      "/" + robot + "/reply", 10, [this](const std_msgs::msg::String::SharedPtr m) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (field(m->data, "id") == id_) {answer_ = m->data;}
      });
  }

private:
  void do_work() override
  {
    const std::string act = get_parameter("action_name").as_string();
    const double t = now().seconds();
    if (started_ < 0.0) {
      started_ = t;
      std::string args;
      for (const auto & a : get_arguments()) {
        args += (args.empty() ? "\"" : ", \"") + a + "\"";
      }
      {
        std::lock_guard<std::mutex> lock(mutex_);
        id_ = act + "-" + std::to_string(++count_) + "-" + std::to_string(static_cast<long long>(t * 1000));
        answer_.clear();
      }
      std_msgs::msg::String m;
      m.data = "{\"id\": \"" + id_ + "\", \"action\": \"" + act + "\", \"args\": [" + args + "]}";
      request_->publish(m);
      sent_ = m.data;
      RCLCPP_INFO(get_logger(), "%s: asked %s", act.c_str(), m.data.c_str());
      return;
    }
    std::string answer;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      answer = answer_;
    }
    if (answer.empty()) {
      // The robot node may have started after the first request: ask again
      // every few seconds until it answers.
      if (t - last_ask_ > 3.0 && t - started_ > 3.0) {
        std_msgs::msg::String m;
        m.data = sent_;
        request_->publish(m);
        last_ask_ = t;
      }
      if (t - started_ > get_parameter("timeout").as_double()) {
        RCLCPP_ERROR(get_logger(), "%s: no answer in %.0f s", act.c_str(), t - started_);
        started_ = -1.0;
        finish(false, 1.0, "no answer");
      }
      send_feedback(0.5, "waiting");
      return;
    }
    const bool ok = field(answer, "ok") == "true";
    const std::string outcome = field(answer, "outcome");
    const std::string why = field(answer, "why");
    RCLCPP_INFO(get_logger(), "%s: %s after %.1f s%s%s", act.c_str(), ok ? "done" : "FAILED", t - started_,
      outcome.empty() ? "" : "; event ", outcome.c_str());
    started_ = -1.0;
    last_ask_ = -1.0;
    finish(ok, 1.0, ok ? (why.empty() ? "done" : why) : why, outcome);
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr request_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr reply_;
  std::mutex mutex_;
  std::string id_, answer_, sent_;
  int count_{0};
  double started_{-1.0};
  double last_ask_{-1.0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RelayAction>();
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
