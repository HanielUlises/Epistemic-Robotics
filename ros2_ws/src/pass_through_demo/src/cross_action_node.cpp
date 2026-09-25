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

// cross(i, t): the carrier takes its load through bay t to the dock.
//
// The epistemic action's precondition is K_i open_t, and the executor checks
// it against the model before dispatching this. The same condition reaches the
// wheels by a second road: the route is the least fixed point
//
//     W = mu Z . dock  v  ( Safe_i ^ <move> Z ),
//     Safe_i = [[ free  v  OR_t ( t ^ K_i open_t ) ]]
//
// over the carrier's own knowledge, so a bay it does not know to be open is not
// in Safe_i and no route passes through it -- whatever the world is like, and
// whatever the executor believes. The two roads agree or the robot does not
// move.
//
// The same computation runs once a second before the action is ever
// dispatched, and its region is published. It is the precondition drawn on the
// floor: while the carrier knows nothing, W stops at the racking block and the
// carrier is not in it; the moment the last announcement makes K_carrier
// open_t2 true, W floods through t2 down to where the carrier is waiting.

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "pass_through_demo/driver.hpp"
#include "plansys2_executor/ActionExecutorClient.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;   // NOLINT(build/namespaces)

namespace
{

std::string join(const std::vector<std::string> & items)
{
  std::string out;
  for (const auto & item : items) {
    out += (out.empty() ? "" : ",") + item;
  }
  return out.empty() ? "none" : out;
}

std::string argument(int argc, char ** argv, const std::string & flag, const std::string & fallback)
{
  for (int i = 1; i + 1 < argc; ++i) {
    if (flag == argv[i]) {return argv[i + 1];}
  }
  return fallback;
}

}  // namespace

class CrossAction : public plansys2::ActionExecutorClient
{
public:
  CrossAction(const std::string & agent, rclcpp::Node::SharedPtr side)
  : plansys2::ActionExecutorClient("cross_" + agent), agent_(agent), side_(side)
  {
    ns_ = side_->declare_parameter<std::string>("ns", "r3");
    const auto floorplan = side_->declare_parameter<std::string>("floorplan", "");
    const auto names = side_->declare_parameter<std::vector<std::string>>("bays", std::vector<std::string>{});
    const auto boxes = side_->declare_parameter<std::vector<double>>("bay_boxes", std::vector<double>{});
    const auto dock = side_->declare_parameter<std::vector<double>>("dock", {0.0, 21.0, 0.6});
    dock_x_ = dock.at(0);
    dock_y_ = dock.at(1);
    dock_r_ = dock.at(2);

    pass_through::Driver::Config config;
    config.agent = agent_;
    config.ns = ns_;
    config.floorplan = floorplan;
    config.bays = pass_through::boxes_from(names, boxes);
    config.speed = side_->declare_parameter<double>("speed", 0.55);
    bays_ = config.bays;
    driver_ = std::make_unique<pass_through::Driver>(side_, config);

    acting_pub_ = side_->create_publisher<std_msgs::msg::String>(
      "/pass_through/acting", rclcpp::QoS(10).transient_local());
    readings_sub_ = side_->create_subscription<std_msgs::msg::String>(
      "/" + ns_ + "/readings", rclcpp::QoS(1).transient_local().reliable(),
      [this](std_msgs::msg::String::SharedPtr msg) {
        try {
          readings_ = nlohmann::json::parse(msg->data);
        } catch (const std::exception &) {
        }
      });

    monitor_ = side_->create_wall_timer(1s, [this]() {monitor();});
  }

private:
  /// The winning region towards the dock, from where the carrier stands, on
  /// what it knows now. Logged when its answer changes.
  void monitor()
  {
    if (driving_) {
      return;
    }
    double x, y, yaw;
    if (!driver_->pose(x, y, yaw)) {
      return;
    }
    const auto a = driver_->assess(dock_x_, dock_y_, dock_r_, true);
    if (!a.ok) {
      return;
    }
    const std::string key = std::string(a.reachable ? "in" : "out") + "/" + join(a.lifted) + "/" +
      join(a.known_open);
    if (key == last_) {
      return;
    }
    last_ = key;
    if (a.reachable) {
      RCLCPP_INFO(
        side_->get_logger(), "[reach] %s in W(dock): |W| = %zu cells after %u iterations; "
        "known open: %s; lifted by K: %s; route %.1f m through %s", agent_.c_str(), a.region,
        a.iterations, join(a.known_open).c_str(), join(a.lifted).c_str(), a.route_length,
        join(a.route_bays).c_str());
    } else {
      RCLCPP_INFO(
        side_->get_logger(), "[reach] %s not in W(dock): |W| = %zu cells after %u iterations; "
        "known open: %s; Safe = %s", agent_.c_str(), a.region, a.iterations,
        join(a.known_open).c_str(), a.formula.c_str());
    }
  }

  void do_work() override
  {
    const auto & args = get_arguments();
    if (args.size() < 2 || !bays_.count(args[1])) {
      finish(false, 0.0, "cross needs (cross <agent> <bay>) with a bay this node knows");
      return;
    }
    const std::string bay = args[1];

    if (!driving_) {
      driving_ = true;
      entered_ = false;
      covered_ = false;
      started_ = now();
      std_msgs::msg::String who;
      who.data = agent_;
      acting_pub_->publish(who);
      const auto a = driver_->assess(dock_x_, dock_y_, dock_r_, true);
      RCLCPP_INFO(
        get_logger(), "[cross] %s sets out for the dock through %s: known open %s, lifted by K "
        "%s, route %.1f m through %s", agent_.c_str(), bay.c_str(), join(a.known_open).c_str(),
        join(a.lifted).c_str(), a.route_length, join(a.route_bays).c_str());
      if (a.reachable &&
        std::find(a.route_bays.begin(), a.route_bays.end(), bay) == a.route_bays.end())
      {
        RCLCPP_WARN(
          get_logger(), "[cross] the route does not pass through %s, which the policy named",
          bay.c_str());
      }
    }

    double x, y, yaw;
    if (driver_->pose(x, y, yaw)) {
      const auto & box = bays_.at(bay);
      if (!entered_ && box.contains(x, y)) {
        entered_ = true;
        RCLCPP_INFO(get_logger(), "[cross] %s enters %s", agent_.c_str(), bay.c_str());
      }
      // The carrier's own scanner reaches into the bay as it approaches. Worth
      // one line: what it knew by elimination it now also sees.
      if (!covered_ && readings_.contains("own") && readings_["own"].contains(bay)) {
        const auto & r = readings_["own"][bay];
        const std::size_t cells = r.value("cells", 0UL);
        const std::size_t seen = cells - r.value("unknown", 0UL);
        if (cells > 0 && seen * 2 >= cells) {
          covered_ = true;
          RCLCPP_INFO(
            get_logger(), "[cross] %s's own scan now covers %s: %zu of %zu cells seen, own map "
            "reads it %s", agent_.c_str(), bay.c_str(), seen, cells,
            r.value("verdict", "unknown").c_str());
        }
      }
    }

    const auto progress = driver_->step_to(dock_x_, dock_y_, dock_r_);
    if (progress == pass_through::Driver::Progress::Arrived) {
      driving_ = false;
      RCLCPP_INFO(
        get_logger(), "[cross] %s at the dock after %.0f s", agent_.c_str(),
        (now() - started_).seconds());
      finish(true, 1.0, "delivered through " + bay);
      return;
    }
    if (progress == pass_through::Driver::Progress::NoRoute &&
      (now() - started_).seconds() > 30.0)
    {
      driving_ = false;
      RCLCPP_ERROR(get_logger(), "[cross] %s has no route to the dock", agent_.c_str());
      finish(false, 0.0, "no route: the carrier does not know a way through");
      return;
    }
    send_feedback(0.5f, "crossing " + bay);
  }

  std::string agent_;
  std::string ns_;
  rclcpp::Node::SharedPtr side_;
  std::unique_ptr<pass_through::Driver> driver_;
  std::map<std::string, pass_through::Box> bays_;
  double dock_x_{0.0}, dock_y_{21.0}, dock_r_{0.6};

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr acting_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr readings_sub_;
  rclcpp::TimerBase::SharedPtr monitor_;
  nlohmann::json readings_;
  std::string last_;
  bool driving_{false};
  bool entered_{false};
  bool covered_{false};
  rclcpp::Time started_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  const auto agent = argument(argc, argv, "--agent", "carrier");

  auto side = std::make_shared<rclcpp::Node>("cross_" + agent + "_driver");
  auto performer = std::make_shared<CrossAction>(agent, side);
  performer->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(side);
  executor.add_node(performer->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
