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

// share-open(i, j, t) and share-shut(i, j, t): i sends j its map.
//
// Epistemically each is a semi-private announcement. i and j observe it
// fully; everyone else observes that i told j something about t, and not
// what. The announcement is "i knows t is open" or "i knows t is shut".
//
// Physically it is a map exchange. The sender's knowledge map -- everything it
// has observed and everything it has been sent -- is fused into the
// receiver's by epistemic_slam, and the receiver's knowledge node reports how
// many cells it newly holds, over the grid and inside each bay.
//
// The two are checked against each other before the performer reports success.
// After a share-shut the receiver's knowledge map must show the load in t;
// after a share-open it must show t observed free. There is a third case, and
// it is the one this domain exists for: the sender may know about t without
// any map containing it, because it knows by elimination. Then the fused map
// says nothing about t, and the performer says so in the log rather than
// pretending the map carried the announcement. The announcement is still
// true, and the executor still applies it: what the sender knew by inference
// is exactly what semi-private announcement of [i] open_t conveys.
//
//     share_action --kind shut     (one process per kind)

#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "epistemic_msgs/srv/absorb_map.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "plansys2_executor/ActionExecutorClient.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;   // NOLINT(build/namespaces)

namespace
{

std::string argument(int argc, char ** argv, const std::string & flag, const std::string & fallback)
{
  for (int i = 1; i + 1 < argc; ++i) {
    if (flag == argv[i]) {return argv[i + 1];}
  }
  return fallback;
}

}  // namespace

class ShareAction : public plansys2::ActionExecutorClient
{
public:
  ShareAction(const std::string & kind, rclcpp::Node::SharedPtr side)
  : plansys2::ActionExecutorClient("share_" + kind), kind_(kind), side_(side)
  {
    const auto agents = side_->declare_parameter<std::vector<std::string>>("agents", std::vector<std::string>{});
    const auto namespaces = side_->declare_parameter<std::vector<std::string>>("namespaces", std::vector<std::string>{});
    transfer_ = side_->declare_parameter<double>("transfer_seconds", 4.0);

    const auto latched = rclcpp::QoS(1).transient_local().reliable();
    for (std::size_t i = 0; i < agents.size() && i < namespaces.size(); ++i) {
      const auto agent = agents[i];
      const auto ns = namespaces[i];
      ns_[agent] = ns;
      maps_subs_.push_back(
        side_->create_subscription<nav_msgs::msg::OccupancyGrid>(
          "/" + ns + "/known_map", latched,
          [this, agent](nav_msgs::msg::OccupancyGrid::SharedPtr msg) {maps_[agent] = msg;}));
      readings_subs_.push_back(
        side_->create_subscription<std_msgs::msg::String>(
          "/" + ns + "/readings", latched,
          [this, agent](std_msgs::msg::String::SharedPtr msg) {
            try {
              readings_[agent] = nlohmann::json::parse(msg->data);
            } catch (const std::exception &) {
            }
          }));
      absorb_[agent] = side_->create_client<epistemic_msgs::srv::AbsorbMap>("/" + ns + "/absorb");
    }
    link_pub_ = side_->create_publisher<std_msgs::msg::String>(
      "/pass_through/link", rclcpp::QoS(10).transient_local());
  }

private:
  enum class Phase {Idle, Sending, Holding, Checking};

  void link(const std::string & text)
  {
    std_msgs::msg::String msg;
    msg.data = text;
    link_pub_->publish(msg);
  }

  void do_work() override
  {
    const auto & args = get_arguments();
    if (args.size() < 3 || !ns_.count(args[0]) || !ns_.count(args[1])) {
      finish(false, 0.0, "share needs (tell_<kind> <from> <to> <bay>) naming known agents");
      return;
    }
    const auto & from = args[0];
    const auto & to = args[1];
    const auto & bay = args[2];
    const std::string expected = kind_ == "open" ? "clear" : "blocked";

    if (phase_ == Phase::Idle) {
      if (!maps_.count(from)) {
        send_feedback(0.0, "waiting for the map of " + from);
        return;
      }
      auto request = std::make_shared<epistemic_msgs::srv::AbsorbMap::Request>();
      request->map = *maps_.at(from);
      request->source = from;
      if (!absorb_.at(to)->service_is_ready()) {
        send_feedback(0.0, "waiting for " + to + " to accept maps");
        return;
      }
      RCLCPP_INFO(
        get_logger(), "[share] %s sends its map to %s: share-%s(%s, %s, %s)", from.c_str(),
        to.c_str(), kind_.c_str(), from.c_str(), to.c_str(), bay.c_str());
      link(from + " " + to + " " + bay + " " + kind_);
      pending_ = absorb_.at(to)->async_send_request(request).share();
      started_ = now();
      phase_ = Phase::Sending;
    }

    if (phase_ == Phase::Sending) {
      if (pending_.wait_for(0s) != std::future_status::ready) {
        if ((now() - started_).seconds() > 15.0) {
          phase_ = Phase::Idle;
          link("");
          finish(false, 0.0, to + " did not answer the exchange");
        }
        return;
      }
      const auto response = pending_.get();
      if (!response->ok) {
        phase_ = Phase::Idle;
        link("");
        finish(false, 0.0, "the exchange was refused: " + response->message);
        return;
      }
      std::uint32_t in_bay = 0;
      for (std::size_t i = 0; i < response->regions.size(); ++i) {
        if (response->regions[i] == bay) {in_bay = response->newly_known_in_region[i];}
      }
      RCLCPP_INFO(
        get_logger(), "[share] %s now holds %u cells it had not observed, %u of them in %s; "
        "%u conflicts", to.c_str(), response->newly_known, in_bay, bay.c_str(),
        response->conflicts);
      phase_ = Phase::Holding;
    }

    if (phase_ == Phase::Holding) {
      // The exchange is instantaneous on a local network. It is held on
      // screen long enough for the link to be seen, which is the only reason
      // for the wait, and it is not counted as anything but that.
      if ((now() - started_).seconds() < transfer_) {
        send_feedback(0.5f, from + " -> " + to);
        return;
      }
      phase_ = Phase::Checking;
    }

    // What the receiver's knowledge map now says about the bay.
    std::string verdict = "unknown";
    if (readings_.count(to) && readings_[to].contains("known") && readings_[to]["known"].contains(bay)) {
      verdict = readings_[to]["known"][bay].value("verdict", "unknown");
    }
    phase_ = Phase::Idle;
    link("");
    if (verdict == expected) {
      RCLCPP_INFO(
        get_logger(), "[share] %s's knowledge map reads %s %s, as announced", to.c_str(),
        bay.c_str(), verdict.c_str());
    } else if (verdict == "unknown") {
      RCLCPP_INFO(
        get_logger(), "[share] %s's knowledge map says nothing about %s: the announcement "
        "carries what %s inferred, not what any map holds", to.c_str(), bay.c_str(), from.c_str());
    } else {
      RCLCPP_ERROR(
        get_logger(), "[share] %s's knowledge map reads %s %s, and the announcement says %s",
        to.c_str(), bay.c_str(), verdict.c_str(), expected.c_str());
      finish(false, 1.0, "the map and the announcement disagree");
      return;
    }
    finish(true, 1.0, from + " told " + to + " about " + bay);
  }

  std::string kind_;
  rclcpp::Node::SharedPtr side_;
  double transfer_{4.0};
  std::map<std::string, std::string> ns_;
  std::map<std::string, nav_msgs::msg::OccupancyGrid::SharedPtr> maps_;
  std::map<std::string, nlohmann::json> readings_;
  std::map<std::string, rclcpp::Client<epistemic_msgs::srv::AbsorbMap>::SharedPtr> absorb_;
  std::vector<rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr> maps_subs_;
  std::vector<rclcpp::Subscription<std_msgs::msg::String>::SharedPtr> readings_subs_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr link_pub_;

  Phase phase_{Phase::Idle};
  std::shared_future<epistemic_msgs::srv::AbsorbMap::Response::SharedPtr> pending_;
  rclcpp::Time started_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  const auto kind = argument(argc, argv, "--kind", "shut");

  auto side = std::make_shared<rclcpp::Node>("share_" + kind + "_side");
  auto performer = std::make_shared<ShareAction>(kind, side);
  performer->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(side);
  executor.add_node(performer->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
