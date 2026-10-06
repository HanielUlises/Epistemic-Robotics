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

// go-view(i): robot i drives to its viewpoint of the beacon, the mouth of t2
// on its own side of the block, and turns to face into the bay.
//
// The epistemic action is public and ontic: at-view(i) becomes true and every
// agent knows it. Arriving, the node checks the claim the observability
// condition of signal rests on, that from where the robot now stands the
// beacon is in line of sight over the floor plan, and logs the answer.

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "coordinated_attack_demo/common.hpp"
#include "pass_through_demo/driver.hpp"
#include "plansys2_executor/ActionExecutorClient.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;   // NOLINT(build/namespaces)

class GoViewAction : public plansys2::ActionExecutorClient
{
public:
  GoViewAction(const std::string & agent, rclcpp::Node::SharedPtr side)
  : plansys2::ActionExecutorClient("go_view_" + agent), agent_(agent), side_(side)
  {
    ns_ = side_->declare_parameter<std::string>("ns", "r1");
    const auto floorplan = side_->declare_parameter<std::string>("floorplan", "");
    const auto view = side_->declare_parameter<std::vector<double>>(
      "viewpoint", std::vector<double>{0.0, -3.26, M_PI / 2});
    const auto beacon = side_->declare_parameter<std::vector<double>>(
      "beacon", std::vector<double>{1.05, 0.0});
    vx_ = view.at(0);
    vy_ = view.at(1);
    vyaw_ = view.at(2);
    bx_ = beacon.at(0);
    by_ = beacon.at(1);
    plan_ = pass_through::load_map_yaml(floorplan);

    pass_through::Driver::Config config;
    config.agent = agent_;
    config.ns = ns_;
    config.floorplan = floorplan;
    driver_ = std::make_unique<pass_through::Driver>(side_, config);

    shot_pub_ = side_->create_publisher<std_msgs::msg::String>(
      "/coordinated_attack/shot", rclcpp::QoS(10).transient_local());
  }

private:
  enum class Phase {Idle, Drive, Face};

  void do_work() override
  {
    if (phase_ == Phase::Idle) {
      phase_ = Phase::Drive;
      started_ = now();
      std_msgs::msg::String shot;
      shot.data = agent_;
      shot_pub_->publish(shot);
      RCLCPP_INFO(
        get_logger(), "[view] %s sets out for its viewpoint of the beacon at (%.2f, %.2f)",
        agent_.c_str(), vx_, vy_);
    }

    if (phase_ == Phase::Drive) {
      if (driver_->step_to(vx_, vy_, 0.2) == pass_through::Driver::Progress::Arrived) {
        phase_ = Phase::Face;
      }
      send_feedback(0.3f, "driving to the viewpoint");
      return;
    }

    if (!driver_->face(vyaw_)) {
      send_feedback(0.8f, "turning to the beacon");
      return;
    }
    double x, y, yaw;
    driver_->pose(x, y, yaw);
    const bool sees = coordinated_attack::line_of_sight(plan_, x, y, bx_, by_);
    RCLCPP_INFO(
      get_logger(), "[view] %s at its viewpoint after %.0f s; beacon in line of sight: %s, "
      "%.1f m", agent_.c_str(), (now() - started_).seconds(), sees ? "yes" : "no",
      std::hypot(bx_ - x, by_ - y));
    phase_ = Phase::Idle;
    if (!sees) {
      finish(false, 1.0, "the beacon is not in line of sight from the viewpoint");
      return;
    }
    finish(true, 1.0, "at the viewpoint");
  }

  std::string agent_;
  std::string ns_;
  rclcpp::Node::SharedPtr side_;
  std::unique_ptr<pass_through::Driver> driver_;
  pass_through::Grid plan_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr shot_pub_;
  double vx_{0.0}, vy_{0.0}, vyaw_{0.0}, bx_{0.0}, by_{0.0};
  Phase phase_{Phase::Idle};
  rclcpp::Time started_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  const auto agent = coordinated_attack::argument(argc, argv, "--agent", "south");

  auto side = std::make_shared<rclcpp::Node>("go_view_" + agent + "_driver");
  auto performer = std::make_shared<GoViewAction>(agent, side);
  performer->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(side);
  executor.add_node(performer->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
