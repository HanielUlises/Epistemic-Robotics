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

// survey(i, t): scout i reads bay t, and reports what its own map says.
//
// The epistemic action is semi-private sensing with two events, e-open and
// e-shut, and the planner branched on them without knowing which will occur.
// This performer is what decides it, and it decides it from one thing: the
// scout's own SLAM map, placed on the floor plan's grid by knowledge_map. Not
// the knowledge map, which may hold what others sent; a sensing action is an
// agent looking, and what it reports is what it saw. Not the simulator, which
// knows where the loads are and is not an agent.
//
// The scout drives to the bay's southern mouth on a route the least fixed
// point finds over what it knows, turns to face into the bay, and waits for
// its map to decide the region: blocked as soon as any cell in it is occupied,
// open only once every cell in it has been observed free. It reports the
// event, and the executor applies it to the model.
//
//     survey_action --agent west   (one process per scout)

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

std::string argument(int argc, char ** argv, const std::string & flag, const std::string & fallback)
{
  for (int i = 1; i + 1 < argc; ++i) {
    if (flag == argv[i]) {return argv[i + 1];}
  }
  return fallback;
}

}  // namespace

class SurveyAction : public plansys2::ActionExecutorClient
{
public:
  SurveyAction(const std::string & agent, rclcpp::Node::SharedPtr side)
  : plansys2::ActionExecutorClient("survey_" + agent), agent_(agent), side_(side)
  {
    ns_ = side_->declare_parameter<std::string>("ns", "r1");
    const auto floorplan = side_->declare_parameter<std::string>("floorplan", "");
    const auto names = side_->declare_parameter<std::vector<std::string>>("bays", std::vector<std::string>{});
    const auto boxes = side_->declare_parameter<std::vector<double>>("bay_boxes", std::vector<double>{});
    const auto regions = side_->declare_parameter<std::vector<double>>("bay_regions", std::vector<double>{});
    const auto mouths = side_->declare_parameter<std::vector<double>>("bay_mouths", std::vector<double>{});
    const auto parking = side_->declare_parameter<std::vector<double>>("bay_parking", std::vector<double>{});
    settle_ = side_->declare_parameter<int>("settle", 3);
    patience_ = side_->declare_parameter<double>("patience", 25.0);
    max_creep_ = side_->declare_parameter<double>("max_creep", 2.3);

    for (std::size_t i = 0; i < names.size(); ++i) {
      mouths_[names[i]] = {mouths.at(2 * i), mouths.at(2 * i + 1)};
      if (parking.size() >= 2 * (i + 1)) {
        parking_[names[i]] = {parking.at(2 * i), parking.at(2 * i + 1)};
      }
    }
    (void)regions;

    pass_through::Driver::Config config;
    config.agent = agent_;
    config.ns = ns_;
    config.floorplan = floorplan;
    config.bays = pass_through::boxes_from(names, boxes);
    driver_ = std::make_unique<pass_through::Driver>(side_, config);

    readings_sub_ = side_->create_subscription<std_msgs::msg::String>(
      "/" + ns_ + "/readings", rclcpp::QoS(1).transient_local().reliable(),
      [this](std_msgs::msg::String::SharedPtr msg) {
        try {
          readings_ = nlohmann::json::parse(msg->data);
          ++readings_seen_;
        } catch (const std::exception &) {
        }
      });
    acting_pub_ = side_->create_publisher<std_msgs::msg::String>(
      "/pass_through/acting", rclcpp::QoS(10).transient_local());
  }

private:
  enum class Phase {Idle, Drive, Face, Read, Retreat, Park};

  void reset()
  {
    phase_ = Phase::Idle;
    same_ = 0;
    last_verdict_.clear();
    creep_from_ = 1e9;
    no_route_logged_ = false;
    outcome_.clear();
    verdict_.clear();
    crept_ = false;
  }

  void do_work() override
  {
    const auto & args = get_arguments();
    if (args.size() < 2 || !mouths_.count(args[1])) {
      finish(false, 0.0, "survey needs (survey <agent> <bay>) with a bay this node knows");
      return;
    }
    const std::string bay = args[1];
    const auto [mx, my] = mouths_.at(bay);

    if (phase_ == Phase::Idle) {
      reset();
      phase_ = Phase::Drive;
      started_ = now();
      std_msgs::msg::String who;
      who.data = agent_;
      acting_pub_->publish(who);
      RCLCPP_INFO(
        get_logger(), "[survey] %s sets out for the mouth of %s at (%.2f, %.2f)",
        agent_.c_str(), bay.c_str(), mx, my);
    }

    if (phase_ == Phase::Drive) {
      const auto progress = driver_->step_to(mx, my, 0.25);
      if (progress == pass_through::Driver::Progress::Arrived) {
        phase_ = Phase::Face;
        RCLCPP_INFO(
          get_logger(), "[survey] %s at the mouth of %s after %.0f s", agent_.c_str(),
          bay.c_str(), (now() - started_).seconds());
      } else if (progress == pass_through::Driver::Progress::NoRoute && !no_route_logged_) {
        no_route_logged_ = true;
        RCLCPP_WARN(
          get_logger(), "[survey] %s has no known route to the mouth of %s", agent_.c_str(),
          bay.c_str());
      }
      send_feedback(0.1f, "driving to the mouth of " + bay);
      return;
    }

    if (phase_ == Phase::Face) {
      // Into the bay, which from the south mouth is due north.
      if (driver_->face(M_PI / 2.0)) {
        phase_ = Phase::Read;
        read_since_ = now();
        seen_at_start_ = readings_seen_;
        double x, y, yaw;
        driver_->pose(x, y, yaw);
        creep_from_ = y;
        RCLCPP_INFO(get_logger(), "[survey] %s faces into %s, reading", agent_.c_str(), bay.c_str());
      }
      send_feedback(0.5f, "turning into " + bay);
      return;
    }

    // Read. A verdict counts when a fresh map has said it `settle` times in a
    // row: the first map after arriving may predate the arrival. The counting
    // runs in the later phases too, harmlessly: nothing reads it there.
    std::string verdict = "unknown";
    std::size_t seen = 0, cells = 0, occupied = 0;
    if (readings_.contains("own") && readings_["own"].contains(bay)) {
      const auto & r = readings_["own"][bay];
      verdict = r.value("verdict", "unknown");
      cells = r.value("cells", 0UL);
      occupied = r.value("occupied", 0UL);
      seen = cells - r.value("unknown", 0UL);
    }
    if (readings_seen_ != counted_) {
      counted_ = readings_seen_;
      if (readings_seen_ > seen_at_start_ && verdict != "unknown" && verdict == last_verdict_) {
        ++same_;
      } else {
        same_ = verdict != "unknown" && readings_seen_ > seen_at_start_ ? 1 : 0;
      }
      last_verdict_ = verdict;
    }

    if (phase_ == Phase::Read && same_ >= settle_) {
      driver_->stop();
      outcome_ = verdict == "clear" ? "e-open" : "e-shut";
      verdict_ = verdict;
      RCLCPP_INFO(
        get_logger(), "[survey] %s read %s on its own map: %s, %zu of %zu cells seen, "
        "%zu occupied -> %s", agent_.c_str(), bay.c_str(), verdict.c_str(), seen, cells,
        occupied, outcome_.c_str());
      phase_ = Phase::Retreat;
    }

    double x, y, yaw;
    driver_->pose(x, y, yaw);

    if (phase_ == Phase::Retreat) {
      // Back out along the bay's axis to where the reading started. The cells
      // behind the robot are the ones it has just driven over.
      if (y > creep_from_ + 0.05) {
        driver_->creep(-0.2);
        send_feedback(0.9f, "backing out of " + bay);
        return;
      }
      driver_->stop();
      phase_ = parking_.count(bay) ? Phase::Park : Phase::Idle;
      if (phase_ == Phase::Idle) {
        const auto outcome = outcome_, verdict_now = verdict_;
        reset();
        finish(true, 1.0, "read " + bay + ": " + verdict_now, outcome);
        return;
      }
      parked_since_ = now();
    }

    if (phase_ == Phase::Park) {
      // Out of the approach to the bay. A scout left standing at the mouth is
      // standing on the carrier's route whenever the bay it read is the open
      // one, and the carrier's laser would stop it there for good.
      const auto [px, py] = parking_.at(bay);
      const auto progress = driver_->step_to(px, py, 0.3);
      if (progress == pass_through::Driver::Progress::Arrived ||
        (now() - parked_since_).seconds() > 25.0)
      {
        driver_->stop();
        RCLCPP_INFO(
          get_logger(), "[survey] %s clear of the approach to %s", agent_.c_str(), bay.c_str());
        const auto outcome = outcome_, verdict_now = verdict_;
        reset();
        finish(true, 1.0, "read " + bay + ": " + verdict_now, outcome);
        return;
      }
      send_feedback(0.95f, "clearing the approach to " + bay);
      return;
    }

    // Undecided: move into the bay, along its axis, over floor the scout's own
    // map already shows free. From the standoff the uprights at the mouth
    // shade two wedges at the far end, and the region is only clear when every
    // cell of it has been seen; past the uprights nothing shades it. A load
    // decides the other way from the standoff, and the scout never moves.
    const double waited = (now() - read_since_).seconds();
    if (waited > 4.0 && verdict == "unknown" && y - creep_from_ < max_creep_) {
      if (!crept_) {
        crept_ = true;
        RCLCPP_INFO(
          get_logger(), "[survey] %s cannot see all of %s from the mouth (%zu of %zu cells); "
          "moving in", agent_.c_str(), bay.c_str(), seen, cells);
      }
      driver_->creep(0.2);
    } else {
      driver_->stop();
    }
    if (waited > patience_ + 30.0) {
      RCLCPP_WARN(
        get_logger(), "[survey] %s could not decide %s: %zu of %zu cells seen", agent_.c_str(),
        bay.c_str(), seen, cells);
      reset();
      finish(false, 1.0, "the bay never settled");
      return;
    }
    send_feedback(0.8f, "reading " + bay);
  }

  std::string agent_;
  std::string ns_;
  rclcpp::Node::SharedPtr side_;
  std::unique_ptr<pass_through::Driver> driver_;
  std::map<std::string, std::pair<double, double>> mouths_;
  std::map<std::string, std::pair<double, double>> parking_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr readings_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr acting_pub_;
  nlohmann::json readings_;
  std::size_t readings_seen_{0};
  std::size_t counted_{0};
  std::size_t seen_at_start_{0};

  Phase phase_{Phase::Idle};
  int settle_{3};
  int same_{0};
  std::string last_verdict_;
  double patience_{25.0};
  double max_creep_{1.0};
  double creep_from_{1e9};
  bool no_route_logged_{false};
  bool crept_{false};
  std::string outcome_;
  std::string verdict_;
  rclcpp::Time started_;
  rclcpp::Time parked_since_;
  rclcpp::Time read_since_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  const auto agent = argument(argc, argv, "--agent", "west");

  auto side = std::make_shared<rclcpp::Node>("survey_" + agent + "_driver");
  auto performer = std::make_shared<SurveyAction>(agent, side);
  performer->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(side);
  executor.add_node(performer->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
