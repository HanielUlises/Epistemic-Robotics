// Copyright 2026 Haniel Ulises
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

// The night porter: a Gazebo actor that walks the routes it is sent.
//
// A route arrives on /porter/route as "<name> <speed> x,y x,y ...": the actor
// walks it at the given speed, its walking animation advanced by the distance
// covered (Gazebo's ActorPlugin does the same), and stands where it ends. Its
// position is published on /porter/state as "<name> <x> <y> <walking>" at
// 10 Hz, and "<name>" on /porter/done when the route is walked. Nothing here
// knows about doors: scripts/porter_node.py swings them as the porter passes.

#include <functional>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo_ros/node.hpp"
#include "ignition/math/Pose3.hh"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace gazebo
{

class PorterPlugin : public ModelPlugin
{
public:
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    actor_ = boost::dynamic_pointer_cast<physics::Actor>(model);
    if (!actor_) {
      gzerr << "PorterPlugin belongs on an actor\n";
      return;
    }
    node_ = gazebo_ros::Node::Get(sdf);
    const auto latched = rclcpp::QoS(10).reliable().transient_local();
    sub_ = node_->create_subscription<std_msgs::msg::String>(
      "/porter/route", latched,
      std::bind(&PorterPlugin::OnRoute, this, std::placeholders::_1));
    state_ = node_->create_publisher<std_msgs::msg::String>("/porter/state", 10);
    done_ = node_->create_publisher<std_msgs::msg::String>("/porter/done", latched);

    physics::TrajectoryInfoPtr info(new physics::TrajectoryInfo());
    info->type = "walking";
    info->duration = 1.0;
    actor_->SetCustomTrajectory(info);

    const auto p = actor_->WorldPose();
    x_ = p.Pos().X();
    y_ = p.Pos().Y();
    heading_ = p.Rot().Yaw() - 1.5707;
    update_ = event::Events::ConnectWorldUpdateBegin(
      std::bind(&PorterPlugin::OnUpdate, this, std::placeholders::_1));
  }

private:
  void OnRoute(const std_msgs::msg::String::SharedPtr msg)
  {
    std::istringstream in(msg->data);
    std::string name;
    double speed = 0.8;
    in >> name >> speed;
    std::vector<std::pair<double, double>> pts;
    std::string tok;
    while (in >> tok) {
      const auto c = tok.find(',');
      if (c == std::string::npos) {continue;}
      pts.emplace_back(std::stod(tok.substr(0, c)), std::stod(tok.substr(c + 1)));
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (name == route_name_) {return;}   // a latched route already walked
    route_name_ = name;
    speed_ = speed;
    route_ = pts;
    next_ = 0;
    walking_ = !route_.empty();
  }

  void OnUpdate(const common::UpdateInfo & info)
  {
    const double dt = last_ < 0.0 ? 0.0 : (info.simTime.Double() - last_);
    last_ = info.simTime.Double();
    std::string finished;
    double moved = 0.0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      double budget = speed_ * dt;
      while (walking_ && budget > 1e-9) {
        const double tx = route_[next_].first, ty = route_[next_].second;
        const double dx = tx - x_, dy = ty - y_;
        const double d = std::hypot(dx, dy);
        if (d > 1e-6) {
          const double target = std::atan2(dy, dx);
          // turn towards the next point at a walker's rate before striding on
          double err = std::remainder(target - heading_, 2.0 * M_PI);
          const double turn = 3.0 * dt;
          if (std::fabs(err) > 0.35) {
            heading_ += std::copysign(std::min(std::fabs(err), turn), err);
            budget *= 0.3;
          } else {
            heading_ = target;
          }
        }
        const double step = std::min(budget, d);
        if (d > 1e-6) {
          x_ += dx / d * step;
          y_ += dy / d * step;
        }
        moved += step;
        budget -= step;
        if (d - step < 1e-6) {
          ++next_;
          if (next_ >= route_.size()) {
            walking_ = false;
            finished = route_name_;
          }
        }
        if (step < 1e-9) {break;}
      }
    }
    ignition::math::Pose3d pose(x_, y_, 1.0, 1.5707, 0.0, heading_ + 1.5707);
    actor_->SetWorldPose(pose, false, false);
    actor_->SetScriptTime(actor_->ScriptTime() + moved * 5.1);

    if (info.simTime.Double() - published_ >= 0.1) {
      published_ = info.simTime.Double();
      std_msgs::msg::String m;
      std::ostringstream out;
      out << route_name_ << " " << x_ << " " << y_ << " " << (walking_ ? 1 : 0);
      m.data = out.str();
      state_->publish(m);
    }
    if (!finished.empty()) {
      std_msgs::msg::String m;
      m.data = finished;
      done_->publish(m);
    }
  }

  physics::ActorPtr actor_;
  gazebo_ros::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr done_;
  event::ConnectionPtr update_;
  std::mutex mutex_;
  std::vector<std::pair<double, double>> route_;
  std::string route_name_;
  std::size_t next_{0};
  bool walking_{false};
  double speed_{0.8};
  double x_{0.0}, y_{0.0}, heading_{0.0};
  double last_{-1.0};
  double published_{-1.0};
};

GZ_REGISTER_MODEL_PLUGIN(PorterPlugin)

}  // namespace gazebo
