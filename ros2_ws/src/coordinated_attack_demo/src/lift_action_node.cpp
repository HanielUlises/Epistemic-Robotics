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

// lift(s): both robots take an end of the load on stand s and raise it
// together.
//
// The epistemic action is public and ontic, with C job(s) in its precondition;
// the executor checks that against the model before this node is ever
// dispatched, and on the radio floor it is never dispatched.
//
// Each robot drives to its own mouth of the stand on a route the least fixed
// point finds over the floor plan, and turns to face into the bay. Neither
// sees the other: the load is between them. When both are in place one start
// time is fixed for both, and at that instant each creeps in under its end of
// the load. The joint start is the commitment the precondition licenses; the
// node logs how far apart the two starts and the two arrivals were. The load
// is then raised, by moving it through gazebo_ros_state: the Waffles have no
// lift, and the frame shows the load coming up off the stand only once both
// robots are under it.

#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "coordinated_attack_demo/common.hpp"
#include "gazebo_msgs/srv/set_entity_state.hpp"
#include "pass_through_demo/driver.hpp"
#include "plansys2_executor/ActionExecutorClient.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;   // NOLINT(build/namespaces)

namespace
{

struct End
{
  std::string agent;
  std::unique_ptr<pass_through::Driver> driver;
  double mouth_x{0.0}, mouth_y{0.0};
  double under_x{0.0}, under_y{0.0};
  double yaw{0.0};
  bool at_mouth{false};
  bool faced{false};
  bool under{false};
  rclcpp::Time started;
  rclcpp::Time arrived;
};

}  // namespace

class LiftAction : public plansys2::ActionExecutorClient
{
public:
  explicit LiftAction(rclcpp::Node::SharedPtr side)
  : plansys2::ActionExecutorClient("lift"), side_(side)
  {
    const auto floorplan = side_->declare_parameter<std::string>("floorplan", "");
    agents_ = side_->declare_parameter<std::vector<std::string>>(
      "agents", std::vector<std::string>{"south", "north"});
    const auto namespaces = side_->declare_parameter<std::vector<std::string>>(
      "namespaces", std::vector<std::string>{"r1", "r2"});
    const auto stands = side_->declare_parameter<std::vector<std::string>>(
      "stands", std::vector<std::string>{"s1", "s2"});
    // Per stand, per agent in order: mouth x, y, under x, y, yaw.
    const auto poses = side_->declare_parameter<std::vector<double>>(
      "poses", std::vector<double>{});
    // Per stand: the load's x, y at rest.
    const auto loads = side_->declare_parameter<std::vector<double>>(
      "loads", std::vector<double>{});
    height_ = side_->declare_parameter<double>("lift_height", 0.12);
    raise_seconds_ = side_->declare_parameter<double>("raise_seconds", 3.0);
    lead_ = side_->declare_parameter<double>("lead_seconds", 1.5);
    creep_ = side_->declare_parameter<double>("creep_speed", 0.15);

    for (std::size_t s = 0; s < stands.size(); ++s) {
      load_[stands[s]] = coordinated_attack::pair_at(loads, s);
      for (std::size_t k = 0; k < agents_.size(); ++k) {
        const std::size_t base = 5 * (s * agents_.size() + k);
        pose_[stands[s]][agents_[k]] = std::vector<double>(
          poses.begin() + base, poses.begin() + base + 5);
      }
    }
    for (std::size_t k = 0; k < agents_.size(); ++k) {
      pass_through::Driver::Config config;
      config.agent = agents_[k];
      config.ns = namespaces.at(k);
      config.floorplan = floorplan;
      ends_[agents_[k]].agent = agents_[k];
      ends_[agents_[k]].driver = std::make_unique<pass_through::Driver>(side_, config);
    }

    state_ = side_->create_client<gazebo_msgs::srv::SetEntityState>("/gazebo/set_entity_state");
    lift_pub_ = side_->create_publisher<std_msgs::msg::String>(
      "/coordinated_attack/lift", rclcpp::QoS(1).transient_local().reliable());
    shot_pub_ = side_->create_publisher<std_msgs::msg::String>(
      "/coordinated_attack/shot", rclcpp::QoS(10).transient_local());
  }

private:
  enum class Phase {Idle, Approach, Wait, Enter, Raise};

  void status(const std::string & stand, const std::string & phase)
  {
    std_msgs::msg::String m;
    m.data = nlohmann::json({{"stand", stand}, {"phase", phase}}).dump();
    lift_pub_->publish(m);
  }

  void set_load(const std::string & stand, double z)
  {
    if (!state_->service_is_ready()) {
      return;
    }
    auto request = std::make_shared<gazebo_msgs::srv::SetEntityState::Request>();
    request->state.name = "load_" + stand;
    request->state.pose.position.x = load_.at(stand).first;
    request->state.pose.position.y = load_.at(stand).second;
    request->state.pose.position.z = z;
    request->state.pose.orientation.w = 1.0;
    request->state.reference_frame = "world";
    state_->async_send_request(request);
  }

  void do_work() override
  {
    const auto & args = get_arguments();
    const std::string stand = args.empty() ? "" : args.back();
    if (!pose_.count(stand)) {
      finish(false, 0.0, "lift needs (lift <agent> <agent> <stand>) with a known stand");
      return;
    }

    if (phase_ == Phase::Idle) {
      phase_ = Phase::Approach;
      started_ = now();
      for (auto & [agent, end] : ends_) {
        const auto & p = pose_.at(stand).at(agent);
        end.mouth_x = p[0];
        end.mouth_y = p[1];
        end.under_x = p[2];
        end.under_y = p[3];
        end.yaw = p[4];
        end.at_mouth = end.faced = end.under = false;
      }
      std_msgs::msg::String shot;
      shot.data = stand;
      shot_pub_->publish(shot);
      status(stand, "approach");
      RCLCPP_INFO(
        get_logger(), "[lift] lift(%s): both robots set out for their mouths of %s",
        stand.c_str(), stand.c_str());
    }

    if (phase_ == Phase::Approach) {
      bool ready = true;
      for (auto & [agent, end] : ends_) {
        if (!end.at_mouth) {
          if (end.driver->step_to(end.mouth_x, end.mouth_y, 0.2) ==
            pass_through::Driver::Progress::Arrived)
          {
            end.at_mouth = true;
            RCLCPP_INFO(
              get_logger(), "[lift] %s at its mouth of %s after %.0f s", agent.c_str(),
              stand.c_str(), (now() - started_).seconds());
          }
        } else if (!end.faced) {
          end.faced = end.driver->face(end.yaw);
        }
        ready = ready && end.at_mouth && end.faced;
      }
      if (ready) {
        phase_ = Phase::Wait;
        go_at_ = now() + rclcpp::Duration::from_seconds(lead_);
        status(stand, "ready");
        RCLCPP_INFO(
          get_logger(), "[lift] both at %s, out of sight of each other; one start for both in "
          "%.1f s", stand.c_str(), lead_);
      }
      send_feedback(0.3f, "driving to the stand");
      return;
    }

    if (phase_ == Phase::Wait) {
      if (now() < go_at_) {
        send_feedback(0.5f, "waiting for the joint start");
        return;
      }
      phase_ = Phase::Enter;
      for (auto & [agent, end] : ends_) {
        end.started = now();
      }
      status(stand, "enter");
      RCLCPP_INFO(get_logger(), "[lift] joint start: both robots drive in under %s",
        stand.c_str());
    }

    if (phase_ == Phase::Enter) {
      bool all = true;
      for (auto & [agent, end] : ends_) {
        if (end.under) {
          end.driver->stop();
          continue;
        }
        double x, y, yaw;
        end.driver->pose(x, y, yaw);
        const double left = end.yaw > 0 ? end.under_y - y : y - end.under_y;
        if (left <= 0.0) {
          end.driver->stop();
          end.under = true;
          end.arrived = now();
          RCLCPP_INFO(
            get_logger(), "[lift] %s under its end of %s after %.1f s", agent.c_str(),
            stand.c_str(), (end.arrived - end.started).seconds());
        } else {
          end.driver->creep(creep_);
          // The laser holds a robot short of the load if it is closer than the
          // target says; that is under it too.
          if ((now() - end.started).seconds() > 25.0) {
            end.driver->stop();
            end.under = true;
            end.arrived = now();
            RCLCPP_WARN(
              get_logger(), "[lift] %s held %.2f m short of its end of %s", agent.c_str(),
              left, stand.c_str());
          }
        }
        all = all && end.under;
      }
      if (!all) {
        send_feedback(0.7f, "driving in under the load");
        return;
      }
      const auto & a = ends_.at(agents_[0]);
      const auto & b = ends_.at(agents_[1]);
      RCLCPP_INFO(
        get_logger(), "[lift] both under %s: starts %.2f s apart, arrivals %.2f s apart",
        stand.c_str(), std::fabs((a.started - b.started).seconds()),
        std::fabs((a.arrived - b.arrived).seconds()));
      phase_ = Phase::Raise;
      raise_from_ = now();
      status(stand, "raise");
    }

    const double t = (now() - raise_from_).seconds() / raise_seconds_;
    set_load(stand, height_ * std::min(1.0, t));
    if (t < 1.0) {
      send_feedback(0.9f, "raising");
      return;
    }
    status(stand, "lifted");
    RCLCPP_INFO(
      get_logger(), "[lift] load_%s raised %.2f m off the stand, %.0f s after lift began",
      stand.c_str(), height_, (now() - started_).seconds());
    phase_ = Phase::Idle;
    finish(true, 1.0, "lifted");
  }

  rclcpp::Node::SharedPtr side_;
  std::vector<std::string> agents_;
  std::map<std::string, End> ends_;
  std::map<std::string, std::map<std::string, std::vector<double>>> pose_;
  std::map<std::string, std::pair<double, double>> load_;
  rclcpp::Client<gazebo_msgs::srv::SetEntityState>::SharedPtr state_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr lift_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr shot_pub_;
  double height_{0.12};
  double raise_seconds_{3.0};
  double lead_{1.5};
  double creep_{0.15};
  Phase phase_{Phase::Idle};
  rclcpp::Time started_;
  rclcpp::Time go_at_;
  rclcpp::Time raise_from_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto side = std::make_shared<rclcpp::Node>("lift_joint");
  auto performer = std::make_shared<LiftAction>(side);
  performer->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(side);
  executor.add_node(performer->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
