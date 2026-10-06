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

// read-order(i, s): the robot that can read the work order drives to the
// terminal and reads it.
//
// The epistemic action is semi-private sensing with two events, e-here (the
// order names s) and e-elsewhere (it does not). The other robot learns that
// the order was read and not what it said. The outcome is decided by the work
// order, which this node is given as a parameter: the order is a record in the
// warehouse's system, not something on the floor, and the robot learns it by
// going to the one terminal it can read it at.

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

class ReadOrderAction : public plansys2::ActionExecutorClient
{
public:
  ReadOrderAction(const std::string & agent, rclcpp::Node::SharedPtr side)
  : plansys2::ActionExecutorClient("read_order_" + agent), agent_(agent), side_(side)
  {
    ns_ = side_->declare_parameter<std::string>("ns", "r1");
    const auto floorplan = side_->declare_parameter<std::string>("floorplan", "");
    const auto terminal = side_->declare_parameter<std::vector<double>>(
      "terminal", std::vector<double>{-13.45, -15.0, M_PI});
    order_ = side_->declare_parameter<std::string>("order", "s1");
    dwell_ = side_->declare_parameter<double>("dwell", 4.0);
    tx_ = terminal.at(0);
    ty_ = terminal.at(1);
    tyaw_ = terminal.at(2);

    pass_through::Driver::Config config;
    config.agent = agent_;
    config.ns = ns_;
    config.floorplan = floorplan;
    driver_ = std::make_unique<pass_through::Driver>(side_, config);

    shot_pub_ = side_->create_publisher<std_msgs::msg::String>(
      "/coordinated_attack/shot", rclcpp::QoS(10).transient_local());
  }

private:
  enum class Phase {Idle, Drive, Face, Read};

  void do_work() override
  {
    const auto & args = get_arguments();
    if (args.size() < 2) {
      finish(false, 0.0, "read_order needs (read_order <agent> <stand>)");
      return;
    }
    const std::string stand = args[1];

    if (phase_ == Phase::Idle) {
      phase_ = Phase::Drive;
      started_ = now();
      std_msgs::msg::String shot;
      shot.data = "terminal";
      shot_pub_->publish(shot);
      RCLCPP_INFO(
        get_logger(), "[order] %s sets out for the work-order terminal at (%.2f, %.2f)",
        agent_.c_str(), tx_, ty_);
    }

    if (phase_ == Phase::Drive) {
      const auto progress = driver_->step_to(tx_, ty_, 0.25);
      if (progress == pass_through::Driver::Progress::Arrived) {
        phase_ = Phase::Face;
        RCLCPP_INFO(
          get_logger(), "[order] %s at the terminal after %.0f s", agent_.c_str(),
          (now() - started_).seconds());
      }
      send_feedback(0.2f, "driving to the terminal");
      return;
    }

    if (phase_ == Phase::Face) {
      if (driver_->face(tyaw_)) {
        phase_ = Phase::Read;
        read_since_ = now();
      }
      send_feedback(0.6f, "turning to the terminal");
      return;
    }

    if ((now() - read_since_).seconds() < dwell_) {
      send_feedback(0.8f, "reading the work order");
      return;
    }
    const std::string outcome = order_ == stand ? "e-here" : "e-elsewhere";
    RCLCPP_INFO(
      get_logger(), "[order] %s reads the work order: it names %s; asked about %s -> %s",
      agent_.c_str(), order_.c_str(), stand.c_str(), outcome.c_str());
    phase_ = Phase::Idle;
    finish(true, 1.0, "the order names " + order_, outcome);
  }

  std::string agent_;
  std::string ns_;
  std::string order_;
  rclcpp::Node::SharedPtr side_;
  std::unique_ptr<pass_through::Driver> driver_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr shot_pub_;
  double tx_{0.0}, ty_{0.0}, tyaw_{0.0};
  double dwell_{4.0};
  Phase phase_{Phase::Idle};
  rclcpp::Time started_;
  rclcpp::Time read_since_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  const auto agent = coordinated_attack::argument(argc, argv, "--agent", "south");

  auto side = std::make_shared<rclcpp::Node>("read_order_" + agent + "_driver");
  auto performer = std::make_shared<ReadOrderAction>(agent, side);
  performer->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(side);
  executor.add_node(performer->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
