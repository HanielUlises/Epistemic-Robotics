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

// go-dock(i): the picker drives to its charging dock, against the west wall
// of the storage floor between two rows of racking.
//
// The epistemic action is public and ontic: at-dock(picker) becomes true and
// both robots know it. Arriving, the node checks the claim the observability
// of the relocation rests on, that from where the robot now stands neither
// bay is in line of sight over the floor plan, and logs the answer.

#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "false_belief_demo/common.hpp"
#include "false_belief_demo/crate.hpp"
#include "pass_through_demo/driver.hpp"
#include "plansys2_executor/ActionExecutorClient.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class GoDockAction : public plansys2::ActionExecutorClient
{
public:
  explicit GoDockAction(rclcpp::Node::SharedPtr side)
  : plansys2::ActionExecutorClient("go_dock"), side_(side)
  {
    agent_ = side_->declare_parameter<std::string>("agent", "picker");
    const auto ns = side_->declare_parameter<std::string>("ns", "r1");
    const auto floorplan = side_->declare_parameter<std::string>("floorplan", "");
    const auto dock = side_->declare_parameter<std::vector<double>>(
      "dock", std::vector<double>{-13.45, -15.0, M_PI});
    const auto names = side_->declare_parameter<std::vector<std::string>>(
      "bays", std::vector<std::string>{"t1", "t3"});
    const auto centres = side_->declare_parameter<std::vector<double>>(
      "bay_centres", std::vector<double>{-10.436, 0.0, 10.436, 0.0});
    dx_ = dock.at(0);
    dy_ = dock.at(1);
    dyaw_ = dock.at(2);
    bays_ = false_belief::bays_from(names, centres);
    plan_ = pass_through::load_map_yaml(floorplan);

    pass_through::Driver::Config config;
    config.agent = agent_;
    config.ns = ns;
    config.floorplan = floorplan;
    driver_ = std::make_unique<pass_through::Driver>(side_, config);
    shot_pub_ = side_->create_publisher<std_msgs::msg::String>(
      "/false_belief/shot", rclcpp::QoS(10).transient_local());
  }

private:
  enum class Phase {Idle, Drive, Face};

  void shot(const std::string & name)
  {
    std_msgs::msg::String m;
    m.data = name;
    shot_pub_->publish(m);
  }

  void do_work() override
  {
    if (phase_ == Phase::Idle) {
      phase_ = Phase::Drive;
      started_ = now();
      shot(agent_);
      RCLCPP_INFO(
        get_logger(), "[dock] %s sets out for its dock at (%.2f, %.2f)", agent_.c_str(), dx_, dy_);
    }
    if (phase_ == Phase::Drive) {
      if (driver_->step_to(dx_, dy_, 0.25) == pass_through::Driver::Progress::Arrived) {
        phase_ = Phase::Face;
      }
      send_feedback(0.4f, "driving to the dock");
      return;
    }
    if (!driver_->face(dyaw_)) {
      send_feedback(0.9f, "turning onto the dock");
      return;
    }
    double x, y, yaw;
    driver_->pose(x, y, yaw);
    std::string seen;
    bool any = false;
    for (const auto & [bay, c] : bays_) {
      const bool sees = false_belief::line_of_sight(plan_, x, y, c.first, c.second, 0.5);
      any = any || sees;
      seen += (seen.empty() ? "" : ", ") + bay + (sees ? " yes" : " no");
    }
    RCLCPP_INFO(
      get_logger(), "[dock] %s at its dock after %.0f s; bays in line of sight: %s",
      agent_.c_str(), (now() - started_).seconds(), seen.c_str());
    shot("dock");
    phase_ = Phase::Idle;
    if (any) {
      finish(false, 1.0, "a bay is in sight of the dock");
      return;
    }
    finish(true, 1.0, "at the dock");
  }

  rclcpp::Node::SharedPtr side_;
  std::string agent_;
  std::unique_ptr<pass_through::Driver> driver_;
  pass_through::Grid plan_;
  std::map<std::string, std::pair<double, double>> bays_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr shot_pub_;
  double dx_{0.0}, dy_{0.0}, dyaw_{0.0};
  Phase phase_{Phase::Idle};
  rclcpp::Time started_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto side = std::make_shared<rclcpp::Node>("go_dock_driver");
  auto performer = std::make_shared<GoDockAction>(side);
  performer->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(side);
  executor.add_node(performer->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
