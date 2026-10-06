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

// look(p, b) and fetch(p, b), one node each (--kind look | fetch): the picker
// drives to its mouth of the bay, on the storage floor, turns to face along
// the bay's axis, and reads its laser straight ahead.
//
// A crate in the bay reads at about the standoff plus the half depth of the
// block; an empty bay reads through to the dispatch floor. The threshold
// between the two is a parameter.
//
// look is sensing: the node reports e-full or e-empty, and the epistemic state
// applies that event. fetch is ontic, and the executor dispatches it only once
// the model has the picker believing the crate is in the bay. The node checks
// that belief against the floor: if the laser reads the bay empty it refuses,
// whatever the model says, and the crate stays where it is. Otherwise the
// picker creeps in, takes the crate onto its top plate, turns, carries it to
// the drop and sets it down.

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
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"

class BayAction : public plansys2::ActionExecutorClient
{
public:
  BayAction(const std::string & kind, rclcpp::Node::SharedPtr side)
  : plansys2::ActionExecutorClient(kind), kind_(kind), side_(side)
  {
    agent_ = side_->declare_parameter<std::string>("agent", "picker");
    const auto ns = side_->declare_parameter<std::string>("ns", "r1");
    const auto floorplan = side_->declare_parameter<std::string>("floorplan", "");
    const auto names = side_->declare_parameter<std::vector<std::string>>(
      "bays", std::vector<std::string>{"t1", "t3"});
    const auto centres = side_->declare_parameter<std::vector<double>>(
      "bay_centres", std::vector<double>{-10.436, 0.0, 10.436, 0.0});
    const auto mouths = side_->declare_parameter<std::vector<double>>(
      "mouths", std::vector<double>{-10.436, -3.26, 10.436, -3.26});
    const auto drop = side_->declare_parameter<std::vector<double>>(
      "drop", std::vector<double>{-5.2, -6.2});
    yaw_in_ = side_->declare_parameter<double>("yaw_in", M_PI / 2);
    threshold_ = side_->declare_parameter<double>("look_threshold", 3.66);
    gap_ = side_->declare_parameter<double>("take_gap", 0.62);
    creep_ = side_->declare_parameter<double>("creep_speed", 0.15);
    const auto crate_size = side_->declare_parameter<std::vector<double>>(
      "crate_size", std::vector<double>{0.45, 0.45, 0.32});
    const auto carry_z = side_->declare_parameter<double>("carry_z", 0.42);
    bays_ = false_belief::bays_from(names, centres);
    mouths_ = false_belief::bays_from(names, mouths);
    drop_x_ = drop.at(0);
    drop_y_ = drop.at(1);

    pass_through::Driver::Config config;
    config.agent = agent_;
    config.ns = ns;
    config.floorplan = floorplan;
    driver_ = std::make_unique<pass_through::Driver>(side_, config);
    crate_ = std::make_unique<false_belief::Crate>(side_, crate_size.at(2), carry_z);
    shot_pub_ = side_->create_publisher<std_msgs::msg::String>(
      "/false_belief/shot", rclcpp::QoS(10).transient_local());
    scan_sub_ = side_->create_subscription<sensor_msgs::msg::LaserScan>(
      "/" + ns + "/scan", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::LaserScan::SharedPtr m) {scan_ = m;});
  }

private:
  enum class Phase {Idle, ToMouth, Face, Read, Enter, TurnOut, ToDrop, Place};

  void shot(const std::string & name)
  {
    std_msgs::msg::String m;
    m.data = name;
    shot_pub_->publish(m);
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
    if (args.size() < 2 || !bays_.count(args[1])) {
      finish(false, 0.0, kind_ + " needs (" + kind_ + " <robot> <bay>) with a known bay");
      return;
    }
    const auto & bay = args[1];

    switch (phase_) {
      case Phase::Idle:
        started_ = now();
        phase_ = Phase::ToMouth;
        shot(agent_);
        RCLCPP_INFO(
          get_logger(), "[%s] %s sets out for its mouth of %s", kind_.c_str(), agent_.c_str(),
          bay.c_str());
        [[fallthrough]];
      case Phase::ToMouth:
        if (driver_->step_to(mouths_.at(bay).first, mouths_.at(bay).second, 0.2) ==
          pass_through::Driver::Progress::Arrived)
        {
          phase_ = Phase::Face;
        }
        send_feedback(0.2f, "driving to " + bay);
        return;
      case Phase::Face:
        if (driver_->face(yaw_in_, 0.04)) {
          phase_ = Phase::Read;
          read_from_ = now();
          shot(bay);
        }
        return;
      case Phase::Read: {
          // A second of scans, so the reading is of the robot at rest.
          if ((now() - read_from_).seconds() < 1.0) {
            return;
          }
          const double range = false_belief::ahead(scan_);
          if (range < 0.0) {
            return;
          }
          const bool full = range < threshold_;
          RCLCPP_INFO(
            get_logger(), "[%s] %s at its mouth of %s after %.0f s; the laser reads %.1f m along "
            "the bay: %s", kind_.c_str(), agent_.c_str(), bay.c_str(),
            (now() - started_).seconds(), std::isfinite(range) ? range : 99.0,
            full ? "the crate is there" : "the bay is empty");
          if (kind_ == "look") {
            phase_ = Phase::Idle;
            const std::string outcome = full ? "e-full" : "e-empty";
            RCLCPP_INFO(get_logger(), "[look] %s -> %s", bay.c_str(), outcome.c_str());
            finish(true, 1.0, full ? "the crate is in " + bay : bay + " is empty", outcome);
            return;
          }
          if (!full) {
            phase_ = Phase::Idle;
            RCLCPP_ERROR(
              get_logger(), "[fetch] %s refuses: there is no crate in %s to fetch",
              agent_.c_str(), bay.c_str());
            finish(false, 1.0, "there is no crate in " + bay);
            return;
          }
          phase_ = Phase::Enter;
          entered_ = now();
          return;
        }
      case Phase::Enter: {
          double x, y, yaw;
          driver_->pose(x, y, yaw);
          const double target = bays_.at(bay).second - gap_;
          if (y < target && (now() - entered_).seconds() < 20.0) {
            driver_->creep(creep_);
            send_feedback(0.5f, "taking the crate");
            return;
          }
          driver_->stop();
          RCLCPP_INFO(
            get_logger(), "[fetch] %s takes the crate from %s after %.0f s", agent_.c_str(),
            bay.c_str(), (now() - started_).seconds());
          phase_ = Phase::TurnOut;
          return;
        }
      case Phase::TurnOut:
        carry();
        if (driver_->face(-yaw_in_)) {
          phase_ = Phase::ToDrop;
          shot(agent_);
        }
        return;
      case Phase::ToDrop:
        carry();
        if (driver_->step_to(drop_x_, drop_y_ + 0.7, 0.25) ==
          pass_through::Driver::Progress::Arrived)
        {
          phase_ = Phase::Place;
          shot("drop");
        }
        send_feedback(0.8f, "carrying the crate to the drop");
        return;
      case Phase::Place:
        crate_->rest("drop", drop_x_, drop_y_);
        RCLCPP_INFO(
          get_logger(), "[fetch] %s sets the crate down at the drop, %.0f s after fetch began",
          agent_.c_str(), (now() - started_).seconds());
        break;
    }
    phase_ = Phase::Idle;
    finish(true, 1.0, "fetched");
  }

  std::string kind_;
  rclcpp::Node::SharedPtr side_;
  std::string agent_;
  std::unique_ptr<pass_through::Driver> driver_;
  std::unique_ptr<false_belief::Crate> crate_;
  std::map<std::string, std::pair<double, double>> bays_, mouths_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr shot_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  sensor_msgs::msg::LaserScan::SharedPtr scan_;
  double drop_x_{0.0}, drop_y_{0.0}, yaw_in_{M_PI / 2}, threshold_{3.66}, gap_{0.62},
    creep_{0.15};
  Phase phase_{Phase::Idle};
  rclcpp::Time started_, entered_, read_from_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  const auto kind = false_belief::argument(argc, argv, "--kind", "look");
  auto side = std::make_shared<rclcpp::Node>(kind + "_driver");
  auto performer = std::make_shared<BayAction>(kind, side);
  performer->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(side);
  executor.add_node(performer->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
