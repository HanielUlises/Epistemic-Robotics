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

// relocate(m, from, to): the mover carries the crate from one bay to the
// other, through the bays' north mouths, on the dispatch floor.
//
// The epistemic action is a relocation the picker does not observe: on the
// told and untold floors the picker maps it onto the crate staying where it
// was, on the doubt floor it relates the two. Which of these the floor
// supports is a matter of where the picker is, and the node checks it: when
// the crate is set down it logs whether either bay is in line of sight of the
// picker over the floor plan, and refuses if one is, since then the move was
// not unobserved.
//
// The mover drives to its mouth of the first bay, creeps in to the crate,
// takes it onto its top plate, turns, carries it to its mouth of the second
// bay, creeps in, sets it down on the bay's axis and leaves for its resting
// place. The crate is moved with the mover through gazebo_ros_state.

#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "false_belief_demo/common.hpp"
#include "false_belief_demo/crate.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "pass_through_demo/driver.hpp"
#include "plansys2_executor/ActionExecutorClient.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class RelocateAction : public plansys2::ActionExecutorClient
{
public:
  explicit RelocateAction(rclcpp::Node::SharedPtr side)
  : plansys2::ActionExecutorClient("relocate"), side_(side)
  {
    agent_ = side_->declare_parameter<std::string>("agent", "mover");
    const auto ns = side_->declare_parameter<std::string>("ns", "r2");
    observer_ = side_->declare_parameter<std::string>("observer", "picker");
    const auto observer_ns = side_->declare_parameter<std::string>("observer_ns", "r1");
    const auto floorplan = side_->declare_parameter<std::string>("floorplan", "");
    const auto names = side_->declare_parameter<std::vector<std::string>>(
      "bays", std::vector<std::string>{"t1", "t3"});
    const auto centres = side_->declare_parameter<std::vector<double>>(
      "bay_centres", std::vector<double>{-10.436, 0.0, 10.436, 0.0});
    const auto mouths = side_->declare_parameter<std::vector<double>>(
      "mouths", std::vector<double>{-10.436, 3.26, 10.436, 3.26});
    const auto rest = side_->declare_parameter<std::vector<double>>(
      "rest", std::vector<double>{6.0, 5.6});
    yaw_in_ = side_->declare_parameter<double>("yaw_in", -M_PI / 2);
    gap_ = side_->declare_parameter<double>("take_gap", 0.62);
    creep_ = side_->declare_parameter<double>("creep_speed", 0.15);
    const auto crate_size = side_->declare_parameter<std::vector<double>>(
      "crate_size", std::vector<double>{0.45, 0.45, 0.32});
    const auto carry_z = side_->declare_parameter<double>("carry_z", 0.42);
    bays_ = false_belief::bays_from(names, centres);
    mouths_ = false_belief::bays_from(names, mouths);
    rx_ = rest.at(0);
    ry_ = rest.at(1);
    plan_ = pass_through::load_map_yaml(floorplan);

    pass_through::Driver::Config config;
    config.agent = agent_;
    config.ns = ns;
    config.floorplan = floorplan;
    driver_ = std::make_unique<pass_through::Driver>(side_, config);
    crate_ = std::make_unique<false_belief::Crate>(side_, crate_size.at(2), carry_z);
    shot_pub_ = side_->create_publisher<std_msgs::msg::String>(
      "/false_belief/shot", rclcpp::QoS(10).transient_local());
    odom_ = side_->create_subscription<nav_msgs::msg::Odometry>(
      "/" + observer_ns + "/odom", rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::SharedPtr m) {
        ox_ = m->pose.pose.position.x;
        oy_ = m->pose.pose.position.y;
        observed_ = true;
      });
  }

private:
  enum class Phase {Idle, ToFrom, FaceFrom, EnterFrom, TurnOut, ToTo, FaceTo, EnterTo, TurnBack,
    Leave};

  void shot(const std::string & name)
  {
    std_msgs::msg::String m;
    m.data = name;
    shot_pub_->publish(m);
  }

  /// Creep along the bay's axis until the robot's centre is gap_ from the
  /// crate's place; the laser holds it short if the crate is nearer.
  bool creep_to(double target_y)
  {
    double x, y, yaw;
    driver_->pose(x, y, yaw);
    const double left = yaw_in_ > 0 ? target_y - y : y - target_y;
    if (left <= 0.0 || (now() - entered_).seconds() > 20.0) {
      driver_->stop();
      return true;
    }
    driver_->creep(creep_);
    return false;
  }

  void carry()
  {
    double x, y, yaw;
    driver_->pose(x, y, yaw);
    crate_->carry(agent_, x, y, yaw);
  }

  void do_work() override
  {
    const auto & args = get_arguments();
    if (args.size() < 3 || !bays_.count(args[1]) || !bays_.count(args[2])) {
      finish(false, 0.0, "relocate needs (relocate <robot> <bay> <bay>) with known bays");
      return;
    }
    const auto & from = args[1];
    const auto & to = args[2];
    const double side = yaw_in_ > 0 ? -1.0 : 1.0;

    switch (phase_) {
      case Phase::Idle:
        started_ = now();
        phase_ = Phase::ToFrom;
        shot(agent_);
        RCLCPP_INFO(
          get_logger(), "[relocate] %s sets out for %s to take the crate to %s",
          agent_.c_str(), from.c_str(), to.c_str());
        [[fallthrough]];
      case Phase::ToFrom:
        if (driver_->step_to(mouths_.at(from).first, mouths_.at(from).second, 0.2) ==
          pass_through::Driver::Progress::Arrived)
        {
          phase_ = Phase::FaceFrom;
          shot(from);
        }
        send_feedback(0.1f, "driving to " + from);
        return;
      case Phase::FaceFrom:
        if (driver_->face(yaw_in_)) {
          phase_ = Phase::EnterFrom;
          entered_ = now();
        }
        return;
      case Phase::EnterFrom:
        if (creep_to(bays_.at(from).second + side * gap_)) {
          phase_ = Phase::TurnOut;
          RCLCPP_INFO(
            get_logger(), "[relocate] %s takes the crate from %s after %.0f s", agent_.c_str(),
            from.c_str(), (now() - started_).seconds());
        }
        send_feedback(0.3f, "taking the crate");
        return;
      case Phase::TurnOut:
        carry();
        if (driver_->face(-yaw_in_)) {
          phase_ = Phase::ToTo;
          shot(agent_);
        }
        return;
      case Phase::ToTo:
        carry();
        if (driver_->step_to(mouths_.at(to).first, mouths_.at(to).second, 0.2) ==
          pass_through::Driver::Progress::Arrived)
        {
          phase_ = Phase::FaceTo;
          shot(to);
        }
        send_feedback(0.5f, "carrying the crate to " + to);
        return;
      case Phase::FaceTo:
        carry();
        if (driver_->face(yaw_in_)) {
          phase_ = Phase::EnterTo;
          entered_ = now();
        }
        return;
      case Phase::EnterTo:
        carry();
        if (creep_to(bays_.at(to).second + side * gap_)) {
          crate_->rest(to, bays_.at(to).first, bays_.at(to).second);
          phase_ = Phase::TurnBack;
          std::string seen = "unknown";
          if (observed_) {
            seen.clear();
            for (const auto & [bay, c] : bays_) {
              const bool sees = false_belief::line_of_sight(plan_, ox_, oy_, c.first, c.second, 0.5);
              witnessed_ = witnessed_ || sees;
              seen += (seen.empty() ? "" : ", ") + bay + (sees ? " yes" : " no");
            }
          }
          RCLCPP_INFO(
            get_logger(), "[relocate] %s sets the crate down in %s after %.0f s; bays in line of "
            "sight of the %s: %s", agent_.c_str(), to.c_str(), (now() - started_).seconds(),
            observer_.c_str(), seen.c_str());
        }
        send_feedback(0.8f, "setting the crate down");
        return;
      case Phase::TurnBack:
        if (driver_->face(-yaw_in_)) {
          phase_ = Phase::Leave;
        }
        return;
      case Phase::Leave:
        if (driver_->step_to(rx_, ry_, 0.3) != pass_through::Driver::Progress::Arrived) {
          send_feedback(0.9f, "leaving the bay");
          return;
        }
        break;
    }
    phase_ = Phase::Idle;
    const bool witnessed = witnessed_;
    witnessed_ = false;
    if (witnessed) {
      finish(false, 1.0, "the relocation was in sight of the " + observer_);
      return;
    }
    finish(true, 1.0, "the crate is in " + to);
  }

  rclcpp::Node::SharedPtr side_;
  std::string agent_, observer_;
  std::unique_ptr<pass_through::Driver> driver_;
  std::unique_ptr<false_belief::Crate> crate_;
  pass_through::Grid plan_;
  std::map<std::string, std::pair<double, double>> bays_, mouths_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr shot_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_;
  double rx_{0.0}, ry_{0.0}, yaw_in_{-M_PI / 2}, gap_{0.62}, creep_{0.15};
  double ox_{0.0}, oy_{0.0};
  bool observed_{false}, witnessed_{false};
  Phase phase_{Phase::Idle};
  rclcpp::Time started_, entered_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto side = std::make_shared<rclcpp::Node>("relocate_driver");
  auto performer = std::make_shared<RelocateAction>(side);
  performer->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(side);
  executor.add_node(performer->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
