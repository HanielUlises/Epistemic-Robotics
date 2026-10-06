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

// signal(i, s): i lights the beacon's tier for stand s.
// signal_to(i, j, s): the same, in the positions domain, where whether the
// listener j saw it is the action's outcome.
//
// The epistemic action is an announcement of K_i job(s) whose audience is
// conditional: an agent at a viewpoint observes it fully, any other is
// oblivious. With both robots at their viewpoints it is a public announcement,
// and job(s) becomes common knowledge in one update.
//
// The beacon is a stack light with one tier per stand, lower for s1 and upper
// for s2; lighting a tier spawns its lit lamp over the post, and the lamp
// stays lit. Before it lights anything, the node reads each robot's pose and
// casts a ray over the floor plan to the post, and logs who has the beacon in
// line of sight. That is the observability condition measured where the
// robots actually stand, beside the one the model assumes, and the two have to
// agree: a robot the model places at a viewpoint, at-view true in every
// designated world, must have the beacon in sight, or the model would record
// it observing an announcement it could not have seen, and the node refuses to
// signal. The first run of this demonstration failed exactly so, before the
// domain said the order is read before the reader goes to its viewpoint.
//
// In the positions domain the signal is an unconfirmed announcement with two
// designated events, e-signal-seen and e-signal-unseen, and the executor needs
// to be told which occurred. The node reports it from the same ray: seen when
// the listener has the beacon in line of sight from where it stands.

#include <chrono>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "coordinated_attack_demo/common.hpp"
#include "gazebo_msgs/srv/spawn_entity.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "plansys2_executor/ActionExecutorClient.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;   // NOLINT(build/namespaces)

class SignalAction : public plansys2::ActionExecutorClient
{
public:
  SignalAction(const std::string & name, rclcpp::Node::SharedPtr side)
  : plansys2::ActionExecutorClient(name), side_(side)
  {
    const auto floorplan = side_->declare_parameter<std::string>("floorplan", "");
    const auto agents = side_->declare_parameter<std::vector<std::string>>(
      "agents", std::vector<std::string>{"south", "north"});
    const auto namespaces = side_->declare_parameter<std::vector<std::string>>(
      "namespaces", std::vector<std::string>{"r1", "r2"});
    const auto beacon = side_->declare_parameter<std::vector<double>>(
      "beacon", std::vector<double>{1.05, 0.0});
    const auto lamps = side_->declare_parameter<std::vector<std::string>>(
      "lamps", std::vector<std::string>{});
    hold_ = side_->declare_parameter<double>("hold", 5.0);
    bx_ = beacon.at(0);
    by_ = beacon.at(1);
    plan_ = pass_through::load_map_yaml(floorplan);

    for (const auto & entry : lamps) {
      const auto eq = entry.find('=');
      if (eq != std::string::npos) {
        lamp_[entry.substr(0, eq)] = entry.substr(eq + 1);
      }
    }
    for (std::size_t k = 0; k < agents.size(); ++k) {
      const auto agent = agents[k];
      agents_.push_back(agent);
      odom_sub_.push_back(side_->create_subscription<nav_msgs::msg::Odometry>(
          "/" + namespaces.at(k) + "/odom", rclcpp::SensorDataQoS(),
          [this, agent](nav_msgs::msg::Odometry::SharedPtr m) {
            pose_[agent] = {m->pose.pose.position.x, m->pose.pose.position.y};
          }));
    }
    state_sub_ = side_->create_subscription<std_msgs::msg::String>(
      "/epistemic_state/state", rclcpp::QoS(10).transient_local(),
      [this](std_msgs::msg::String::SharedPtr m) {on_state(m->data);});
    spawn_ = side_->create_client<gazebo_msgs::srv::SpawnEntity>("/spawn_entity");
    beacon_pub_ = side_->create_publisher<std_msgs::msg::String>(
      "/coordinated_attack/beacon", rclcpp::QoS(1).transient_local().reliable());
    shot_pub_ = side_->create_publisher<std_msgs::msg::String>(
      "/coordinated_attack/shot", rclcpp::QoS(10).transient_local());
  }

private:
  /// Which agents the model places at a viewpoint: at-view true in every
  /// designated world.
  void on_state(const std::string & text)
  {
    try {
      const auto payload = nlohmann::json::parse(text);
      if (!payload.contains("model")) {return;}
      const auto & model = payload["model"];
      std::set<std::string> at_view;
      for (const auto & agent : agents_) {
        bool everywhere = !model["designated"].empty();
        for (const auto & w : model["designated"]) {
          const auto & labels = model["labels"][w.get<std::string>()];
          everywhere = everywhere &&
            std::find(labels.begin(), labels.end(), "at-view_" + agent) != labels.end();
        }
        if (everywhere) {at_view.insert(agent);}
      }
      at_view_ = at_view;
    } catch (const std::exception &) {
    }
  }

  void do_work() override
  {
    const auto & args = get_arguments();
    const bool named = args.size() >= 3;
    const std::string stand = args.empty() ? "" : args.back();
    if (args.size() < 2 || !lamp_.count(stand)) {
      finish(false, 0.0, "signal needs (signal <agent> <stand>) or (signal_to <agent> <agent> "
        "<stand>) with a stand that has a lamp");
      return;
    }
    const auto & who = args[0];
    const std::string listener = named ? args[1] : "";

    if (!lit_) {
      std_msgs::msg::String shot;
      shot.data = "beacon";
      shot_pub_->publish(shot);

      std::string seen, disagree;
      for (const auto & agent : agents_) {
        if (!pose_.count(agent)) {continue;}
        const auto [x, y] = pose_.at(agent);
        const bool sees = coordinated_attack::line_of_sight(plan_, x, y, bx_, by_);
        char dist[16];
        std::snprintf(dist, sizeof(dist), "%.1f", std::hypot(bx_ - x, by_ - y));
        seen += (seen.empty() ? "" : ", ") + agent + " " + (sees ? "yes" : "no") +
          (sees ? std::string(" (") + dist + " m)" : std::string());
        if (at_view_.count(agent) && !sees) {
          disagree += (disagree.empty() ? "" : ", ") + agent;
        }
      }
      if (!disagree.empty()) {
        RCLCPP_ERROR(
          get_logger(), "[signal] refused: the model places %s at a viewpoint, and from where "
          "it stands the beacon is not in line of sight (%s)", disagree.c_str(), seen.c_str());
        finish(false, 0.0, "the model and the floor disagree about who sees the beacon");
        return;
      }
      RCLCPP_INFO(
        get_logger(), "[signal] %s lights the %s tier of the beacon; in line of sight of it: %s",
        who.c_str(), stand.c_str(), seen.c_str());
      if (named) {
        bool sees = false;
        if (pose_.count(listener)) {
          const auto [x, y] = pose_.at(listener);
          sees = coordinated_attack::line_of_sight(plan_, x, y, bx_, by_);
        }
        outcome_ = sees ? "e-signal-seen" : "e-signal-unseen";
        RCLCPP_INFO(
          get_logger(), "[signal] %s %s it -> %s", listener.c_str(),
          sees ? "has the beacon in sight and sees" : "is out of sight of the beacon and misses",
          outcome_.c_str());
      }

      std::ifstream in(lamp_.at(stand));
      std::stringstream xml;
      xml << in.rdbuf();
      auto request = std::make_shared<gazebo_msgs::srv::SpawnEntity::Request>();
      request->name = "beacon_lamp_" + stand;
      request->xml = xml.str();
      request->initial_pose.position.x = bx_;
      request->initial_pose.position.y = by_;
      request->initial_pose.orientation.w = 1.0;
      if (spawn_->service_is_ready()) {
        spawn_->async_send_request(request);
      } else {
        RCLCPP_WARN(get_logger(), "[signal] /spawn_entity is not up; the lamp is not drawn");
      }
      std_msgs::msg::String lit;
      lit.data = "lit " + stand;
      beacon_pub_->publish(lit);
      lit_ = true;
      since_ = now();
    }

    if ((now() - since_).seconds() < hold_) {
      send_feedback(0.5f, "the beacon shows " + stand);
      return;
    }
    lit_ = false;
    if (named) {
      finish(true, 1.0, "the beacon shows " + stand, outcome_);
      return;
    }
    finish(true, 1.0, "the beacon shows " + stand);
  }

  rclcpp::Node::SharedPtr side_;
  pass_through::Grid plan_;
  std::vector<std::string> agents_;
  std::map<std::string, std::string> lamp_;
  std::map<std::string, std::pair<double, double>> pose_;
  std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> odom_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_sub_;
  std::set<std::string> at_view_;
  rclcpp::Client<gazebo_msgs::srv::SpawnEntity>::SharedPtr spawn_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr beacon_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr shot_pub_;
  double bx_{0.0}, by_{0.0};
  double hold_{5.0};
  bool lit_{false};
  std::string outcome_;
  rclcpp::Time since_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  // signal for the coordinated-attack domain, signal_to for the positions one.
  const auto name = coordinated_attack::argument(argc, argv, "--name", "signal");
  auto side = std::make_shared<rclcpp::Node>(name + "_beacon");
  auto performer = std::make_shared<SignalAction>(name, side);
  performer->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(side);
  executor.add_node(performer->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
