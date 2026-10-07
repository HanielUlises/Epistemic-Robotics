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

// The muddy robots' crew: every robot's driver, and the two actions of the
// domain.
//
// One process owns the robots' motion, so that no two nodes ever drive one
// robot. It does three things with them.
//
//   assemble   on request, before planning: every robot drives from its
//              station to its place on the muster ring and turns to face the
//              centre. The epistemic model starts here, where each robot
//              sees every other robot's lamp.
//   bell       the bell rings. The node reads the model the epistemic state
//              published after the last action, finds the actual world, the
//              designated world whose lit lamps are the lamps the diagnostics
//              lit, and the robots that know there that they are faulty:
//              faulty(i) in every world i considers possible from it. Those
//              leave for the calibration bay, and the outcome is e-leave; if
//              there are none, nobody moves and the outcome is e-stay. The
//              designated worlds are the patterns the executor, which does
//              not see the lamps, cannot rule out; what a robot knows is what
//              it knows at the actual one. Before any robot leaves, the node
//              checks that its lamp is in fact lit: the model saying a robot
//              knows it is faulty when it is not would be a model that is
//              wrong about the floor, and the node refuses. It refuses too
//              when no designated world matches the lamps, or no model newer
//              than the last action has arrived.
//   dismiss    on request, after the policy: every robot does what it now
//              knows. One that knows it is faulty and is not yet in the bay
//              goes there; one that knows it is not goes back to its station;
//              one that does not know stays where it is, and is reported.
//
// announce is the supervisor speaking over the public address: the lamp on
// the PA mast lights and stays lit, and every robot hears it. It moves
// nobody.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "gazebo_msgs/srv/delete_entity.hpp"
#include "gazebo_msgs/srv/spawn_entity.hpp"
#include "pass_through_demo/driver.hpp"
#include "plansys2_executor/ActionExecutorClient.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;   // NOLINT(build/namespaces)

namespace
{

struct Pose
{
  double x{0.0}, y{0.0}, yaw{0.0};
};

struct Point
{
  double x{0.0}, y{0.0}, z{0.0};
};

/// x, y, yaw triples from a flat list.
std::vector<Pose> poses_of(const std::vector<double> & flat)
{
  std::vector<Pose> out;
  for (std::size_t i = 0; i + 3 <= flat.size(); i += 3) {
    out.push_back({flat[i], flat[i + 1], flat[i + 2]});
  }
  return out;
}

std::string read_file(const std::string & path)
{
  std::ifstream in(path);
  std::stringstream text;
  text << in.rdbuf();
  return text.str();
}

}  // namespace

/// The robots, and where each is going.
class Crew
{
public:
  enum class Goal {None, Muster, Bay, Station};

  struct Robot
  {
    std::string agent;
    std::unique_ptr<pass_through::Driver> driver;
    Pose muster, station;
    Goal goal{Goal::None};
    std::vector<Pose> legs;      ///< the way to the goal, the goal last
    std::size_t leg{0};
    bool faced{false};
    bool lit{false};
    rclcpp::Time since;
    rclcpp::Time start_at;
  };

  explicit Crew(rclcpp::Node::SharedPtr node)
  : node_(node)
  {
    const auto floorplan = node_->declare_parameter<std::string>("floorplan", "");
    agents_ = node_->declare_parameter<std::vector<std::string>>(
      "agents", std::vector<std::string>{"r1", "r2", "r3", "r4"});
    const auto namespaces = node_->declare_parameter<std::vector<std::string>>(
      "namespaces", std::vector<std::string>{"r1", "r2", "r3", "r4"});
    const auto muster = poses_of(node_->declare_parameter<std::vector<double>>(
        "muster", std::vector<double>{}));
    bay_ = poses_of(node_->declare_parameter<std::vector<double>>(
        "bay", std::vector<double>{}));
    const auto centre = node_->declare_parameter<std::vector<double>>(
      "centre", std::vector<double>{0.0, 0.0});
    centre_ = {centre.at(0), centre.at(1), 0.0};
    const auto stations = poses_of(node_->declare_parameter<std::vector<double>>(
        "stations", std::vector<double>{}));
    const auto faults = node_->declare_parameter<std::vector<std::string>>(
      "faults", std::vector<std::string>{});
    for (std::size_t k = 0; k < agents_.size(); ++k) {
      auto & r = robots_[agents_[k]];
      r.agent = agents_[k];
      pass_through::Driver::Config config;
      config.agent = agents_[k];
      config.ns = namespaces.at(k);
      config.floorplan = floorplan;
      r.driver = std::make_unique<pass_through::Driver>(node_, config);
      r.muster = muster.at(k);
      r.station = stations.at(k);
      r.lit = std::find(faults.begin(), faults.end(), agents_[k]) != faults.end();
    }
    state_sub_ = node_->create_subscription<std_msgs::msg::String>(
      "/epistemic_state/state", rclcpp::QoS(10).transient_local(),
      [this](std_msgs::msg::String::SharedPtr m) {on_state(m->data);});
    request_sub_ = node_->create_subscription<std_msgs::msg::String>(
      "/muddy_robots/request", rclcpp::QoS(10).reliable(),
      [this](std_msgs::msg::String::SharedPtr m) {on_request(m->data);});
    crew_pub_ = node_->create_publisher<std_msgs::msg::String>(
      "/muddy_robots/crew", rclcpp::QoS(10).transient_local().reliable());
    shot_pub_ = node_->create_publisher<std_msgs::msg::String>(
      "/muddy_robots/shot", rclcpp::QoS(10).transient_local());
    timer_ = node_->create_wall_timer(100ms, [this]() {step();});
  }

  /// The actual world: the designated world whose lit lamps are the lamps
  /// the diagnostics lit. The designated worlds are every fault pattern the
  /// executor cannot yet rule out; which of them is the case is what the
  /// floor says, and what a robot knows is what it knows there.
  std::string actual() const
  {
    if (model_.is_null() || !model_.contains("designated") || !model_.contains("labels")) {
      return "";
    }
    for (const auto & w : model_.at("designated")) {
      const auto & labels = model_.at("labels").at(w.get<std::string>());
      bool same = true;
      for (const auto & agent : agents_) {
        const bool has = std::find(labels.begin(), labels.end(), "faulty_" + agent) !=
          labels.end();
        same = same && has == robots_.at(agent).lit;
      }
      if (same) {
        return w.get<std::string>();
      }
    }
    return "";
  }

  /// What each robot knows of its own lamp, at the actual world: "faulty",
  /// "clean", or "" when it does not know.
  ///
  /// Empty when there is no model, no designated world matches the lamps,
  /// or the model lacks a robot's relation at that world: none of those is
  /// a robot not knowing, and callers refuse on it.
  std::map<std::string, std::string> knowledge() const
  {
    std::map<std::string, std::string> out;
    const auto w = actual();
    if (w.empty()) {
      return out;
    }
    try {
      for (const auto & agent : agents_) {
        const std::string atom = "faulty_" + agent;
        const auto & seen = model_.at("relations").at(agent).at(w);
        if (seen.empty()) {
          return {};
        }
        bool always = true, never = true;
        for (const auto & v : seen) {
          const auto & labels = model_.at("labels").at(v.get<std::string>());
          const bool has = std::find(labels.begin(), labels.end(), atom) != labels.end();
          always = always && has;
          never = never && !has;
        }
        out[agent] = always ? "faulty" : never ? "clean" : "";
      }
    } catch (const std::exception &) {
      return {};
    }
    return out;
  }

  /// Whether a model has arrived since `t`: the epistemic state publishes
  /// after it applies an action, and a bell must read the model with the
  /// previous action in it.
  bool fresh_since(const rclcpp::Time & t) const
  {
    return !model_.is_null() && state_at_.nanoseconds() > 0 && state_at_ > t;
  }

  void finished() {finished_at_ = node_->now();}
  rclcpp::Time finished_at() const {return finished_at_;}

  bool lit(const std::string & agent) const {return robots_.at(agent).lit;}

  /// Sends a robot somewhere, after `delay` seconds. A robot leaving the
  /// muster ring first steps out of it along its radius, so that it does not
  /// drive across the ring through the others; the drivers plan over the
  /// floor plan, which has no robots on it. A robot sent to the bay takes the
  /// westmost free place, so that one arriving later never has to pass one
  /// already parked.
  void send(const std::string & agent, Goal goal, double delay = 0.0)
  {
    auto & r = robots_.at(agent);
    r.legs.clear();
    if (where(agent) == Goal::Muster && goal != Goal::Muster) {
      const double dx = r.muster.x - centre_.x, dy = r.muster.y - centre_.y;
      const double len = std::hypot(dx, dy);
      r.legs.push_back({r.muster.x + 0.9 * dx / len, r.muster.y + 0.9 * dy / len, 0.0});
    }
    if (goal == Goal::Muster) {
      r.legs.push_back(r.muster);
    } else if (goal == Goal::Station) {
      r.legs.push_back(r.station);
    } else {
      std::size_t k = 0;
      while (k < bay_.size() && bay_taken_.count(k)) {
        ++k;
      }
      bay_taken_.insert(k);
      r.legs.push_back(bay_.at(std::min(k, bay_.size() - 1)));
    }
    r.leg = 0;
    r.goal = goal;
    r.faced = false;
    r.since = node_->now() + rclcpp::Duration::from_seconds(delay);
    r.start_at = r.since;
  }

  /// How far a robot's place on the ring is from the bay: the order in which
  /// robots leave for it.
  double to_bay(const std::string & agent) const
  {
    const auto & r = robots_.at(agent);
    return std::hypot(r.muster.x - bay_.front().x, r.muster.y - bay_.front().y);
  }

  bool arrived(const std::string & agent) const
  {
    const auto & r = robots_.at(agent);
    return r.goal == Goal::None;
  }

  bool idle() const
  {
    return std::all_of(robots_.begin(), robots_.end(),
             [](const auto & kv) {return kv.second.goal == Goal::None;});
  }

  Goal where(const std::string & agent) const {return place_.count(agent) ? place_.at(agent) :
           Goal::Station;}

  void shot(const std::string & name)
  {
    std_msgs::msg::String m;
    m.data = name;
    shot_pub_->publish(m);
  }

  void say(const std::string & what)
  {
    std_msgs::msg::String m;
    m.data = what;
    crew_pub_->publish(m);
  }

  rclcpp::Logger logger() const {return node_->get_logger();}
  rclcpp::Time now() const {return node_->now();}
  const std::vector<std::string> & agents() const {return agents_;}

private:
  void on_state(const std::string & text)
  {
    try {
      const auto payload = nlohmann::json::parse(text);
      if (payload.contains("model")) {
        model_ = payload["model"];
        state_at_ = node_->now();
      }
    } catch (const std::exception &) {
    }
  }

  void on_request(const std::string & what)
  {
    if (what == "assemble") {
      RCLCPP_INFO(logger(), "[crew] the shift is over: every robot drives to the muster point");
      for (const auto & a : agents_) {
        send(a, Goal::Muster);
      }
      pending_ = "assembled";
      shot("muster");
    } else if (what == "dismiss") {
      const auto know = knowledge();
      if (know.empty()) {
        RCLCPP_ERROR(logger(), "[crew] refused to dismiss: no model, or no designated world "
          "matches the lamps");
        pending_ = "dismissed: refused, the model does not match the floor";
        return;
      }
      std::string summary, home;
      int leaving = 0;
      // Nearest the bay first, as at the bell.
      auto order = agents_;
      std::sort(order.begin(), order.end(),
        [this](const auto & a, const auto & b) {return to_bay(a) < to_bay(b);});
      for (const auto & a : order) {
        const auto k = know.count(a) ? know.at(a) : std::string{};
        const bool lamp = robots_.at(a).lit;
        if ((k == "faulty" && !lamp) || (k == "clean" && lamp)) {
          // The floor has the last word, as at the bell.
          RCLCPP_ERROR(logger(), "[crew] refused: the model says %s knows it is %s, and its "
            "lamp is %s", a.c_str(), k == "faulty" ? "faulty" : "not", robots_.at(a).lit ?
            "lit" : "dark");
          summary += (summary.empty() ? "" : "; ") + a + ": the model is wrong about it";
          continue;
        }
        if (k == "faulty" && where(a) != Goal::Bay) {
          send(a, Goal::Bay, 5.0 * static_cast<double>(leaving++));
          summary += (summary.empty() ? "" : "; ") + a + " knows it is faulty: to the bay";
        } else if (k == "faulty") {
          summary += (summary.empty() ? "" : "; ") + a + " is in the bay";
        } else if (k == "clean") {
          send(a, Goal::Station);
          if (home.empty()) {home = a;}
          summary += (summary.empty() ? "" : "; ") + a + " knows it is not: back to work";
        } else {
          summary += (summary.empty() ? "" : "; ") + a + " does not know: it stays";
        }
      }
      RCLCPP_INFO(logger(), "[crew] dismissed, each on what it knows: %s", summary.c_str());
      pending_ = "dismissed: " + summary;
      // The camera follows the first robot that goes back to work.
      shot(home.empty() ? "hall" : home);
    }
  }

  void step()
  {
    for (auto & [agent, r] : robots_) {
      if (r.goal == Goal::None || node_->now() < r.start_at) {
        continue;
      }
      const Pose & target = r.legs.at(r.leg);
      const bool last = r.leg + 1 == r.legs.size();
      if (!r.faced) {
        const auto progress = r.driver->step_to(target.x, target.y, last ? 0.12 : 0.3);
        if (progress != pass_through::Driver::Progress::Arrived) {
          continue;
        }
        if (!last) {
          ++r.leg;
          continue;
        }
        r.faced = r.driver->face(target.yaw);
        if (!r.faced) {
          continue;
        }
      }
      r.driver->stop();
      place_[agent] = r.goal;
      const char * name = r.goal == Goal::Muster ? "its place at the muster point" :
        r.goal == Goal::Bay ? "the calibration bay" : "its station";
      RCLCPP_INFO(
        logger(), "[crew] %s at %s after %.0f s", agent.c_str(), name,
        (node_->now() - r.since).seconds());
      r.goal = Goal::None;
    }
    if (!pending_.empty() && idle()) {
      say(pending_);
      if (pending_ == "assembled") {
        RCLCPP_INFO(logger(), "[crew] all at the muster point, each in sight of every other "
          "robot's lamp");
      }
      pending_.clear();
    }
  }

  rclcpp::Node::SharedPtr node_;
  std::vector<std::string> agents_;
  std::map<std::string, Robot> robots_;
  std::map<std::string, Goal> place_;
  rclcpp::Time state_at_{0, 0, RCL_ROS_TIME};
  rclcpp::Time finished_at_{0, 0, RCL_ROS_TIME};
  std::vector<Pose> bay_;
  std::set<std::size_t> bay_taken_;
  Pose centre_;
  nlohmann::json model_;
  std::string pending_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr request_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr crew_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr shot_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

/// Lamps spawned when something lights: the PA, and the bell while it rings.
class Lamps
{
public:
  Lamps(rclcpp::Node::SharedPtr node, const std::string & dir)
  : node_(node), dir_(dir)
  {
    spawn_ = node_->create_client<gazebo_msgs::srv::SpawnEntity>("/spawn_entity");
    delete_ = node_->create_client<gazebo_msgs::srv::DeleteEntity>("/delete_entity");
  }

  void light(const std::string & name, const std::string & model, double x, double y, double z)
  {
    if (!spawn_->service_is_ready()) {
      RCLCPP_WARN(node_->get_logger(), "/spawn_entity is not up; %s is not drawn", name.c_str());
      return;
    }
    auto request = std::make_shared<gazebo_msgs::srv::SpawnEntity::Request>();
    request->name = name;
    request->xml = read_file(dir_ + "/" + model);
    request->initial_pose.position.x = x;
    request->initial_pose.position.y = y;
    request->initial_pose.position.z = z;
    request->initial_pose.orientation.w = 1.0;
    spawn_->async_send_request(request);
  }

  void put_out(const std::string & name)
  {
    if (!delete_->service_is_ready()) {
      return;
    }
    auto request = std::make_shared<gazebo_msgs::srv::DeleteEntity::Request>();
    request->name = name;
    delete_->async_send_request(request);
  }

private:
  rclcpp::Node::SharedPtr node_;
  std::string dir_;
  rclcpp::Client<gazebo_msgs::srv::SpawnEntity>::SharedPtr spawn_;
  rclcpp::Client<gazebo_msgs::srv::DeleteEntity>::SharedPtr delete_;
};

class AnnounceAction : public plansys2::ActionExecutorClient
{
public:
  AnnounceAction(Crew & crew, Lamps & lamps, rclcpp::Node::SharedPtr side)
  : plansys2::ActionExecutorClient("announce"), crew_(crew), lamps_(lamps)
  {
    const auto pa = side->declare_parameter<std::vector<double>>("pa", std::vector<double>{0, 0,
        2.5});
    pa_ = {pa.at(0), pa.at(1), pa.at(2)};
    pa_pub_ = side->create_publisher<std_msgs::msg::String>(
      "/muddy_robots/pa", rclcpp::QoS(1).transient_local().reliable());
  }

private:
  void do_work() override
  {
    if (!started_) {
      started_ = true;
      since_ = now();
      crew_.shot("pa");
    }
    // The camera turns to the mast first, so that the lamp lights in view.
    const double held = (now() - since_).seconds();
    if (!speaking_ && held > 2.5) {
      speaking_ = true;
      lamps_.light("pa_lamp", "pa_lamp.sdf", pa_.x, pa_.y, pa_.z);
      std_msgs::msg::String m;
      m.data = "at least one of you is faulty";
      pa_pub_->publish(m);
      RCLCPP_INFO(
        get_logger(), "[pa] the supervisor, over the public address: \"at least one of you is "
        "faulty\"; every robot hears it, and hears that every other robot does");
    }
    if (held < 8.5) {
      send_feedback(0.5f, "speaking");
      return;
    }
    started_ = speaking_ = false;
    crew_.finished();
    finish(true, 1.0, "announced");
  }

  Crew & crew_;
  Lamps & lamps_;
  Point pa_;
  bool started_{false};
  bool speaking_{false};
  rclcpp::Time since_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pa_pub_;
};

class BellAction : public plansys2::ActionExecutorClient
{
public:
  BellAction(Crew & crew, Lamps & lamps, rclcpp::Node::SharedPtr side)
  : plansys2::ActionExecutorClient("bell"), crew_(crew), lamps_(lamps)
  {
    const auto bell = side->declare_parameter<std::vector<double>>(
      "bell", std::vector<double>{0, 0, 0.6});
    bell_ = {bell.at(0), bell.at(1), bell.at(2)};
    dwell_ = side->declare_parameter<double>("bell_dwell", 7.0);
    bell_pub_ = side->create_publisher<std_msgs::msg::String>(
      "/muddy_robots/bell", rclcpp::QoS(10).transient_local().reliable());
  }

private:
  void do_work() override
  {
    if (!ringing_) {
      // The model must have the previous action in it.
      if (!crew_.fresh_since(crew_.finished_at())) {
        if (!waiting_) {
          waiting_ = true;
          wait_from_ = now();
        }
        if ((now() - wait_from_).seconds() < 10.0) {
          send_feedback(0.0f, "waiting for the model after the last action");
          return;
        }
        RCLCPP_ERROR(get_logger(), "[bell] refused: no model newer than the last action");
        waiting_ = false;
        finish(false, 0.0, "no fresh model");
        return;
      }
      waiting_ = false;
      const auto know = crew_.knowledge();
      if (know.empty()) {
        RCLCPP_ERROR(get_logger(), "[bell] refused: no designated world matches the lamps");
        finish(false, 0.0, "no designated world matches the lamps");
        return;
      }
      ringing_ = true;
      ++count_;
      since_ = now();
      leavers_.clear();
      crew_.shot("overhead");
      lamps_.light("bell_lamp", "bell_lamp.sdf", bell_.x, bell_.y, bell_.z);
      for (const auto & a : crew_.agents()) {
        if (know.count(a) && know.at(a) == "faulty") {
          leavers_.push_back(a);
        }
      }
      // The floor has the last word: a robot the model says knows its lamp is
      // lit must have a lit lamp.
      for (const auto & a : leavers_) {
        if (!crew_.lit(a)) {
          RCLCPP_ERROR(
            get_logger(), "[bell] refused: the model says %s knows it is faulty, and its lamp "
            "is not lit", a.c_str());
          lamps_.put_out("bell_lamp");
          ringing_ = false;
          finish(false, 0.0, "the model and the floor disagree");
          return;
        }
      }
      // They leave one by one, nearest the bay first, five seconds apart.
      std::sort(leavers_.begin(), leavers_.end(),
        [this](const auto & a, const auto & b) {return crew_.to_bay(a) < crew_.to_bay(b);});
      std::string who;
      for (std::size_t k = 0; k < leavers_.size(); ++k) {
        who += (who.empty() ? "" : ", ") + leavers_[k];
        crew_.send(leavers_[k], Crew::Goal::Bay, 5.0 * static_cast<double>(k));
      }
      std_msgs::msg::String m;
      m.data = nlohmann::json({{"bell", count_}, {"leave", leavers_}}).dump();
      bell_pub_->publish(m);
      if (leavers_.empty()) {
        RCLCPP_INFO(
          get_logger(), "[bell] bell %d: no robot knows whether it is faulty, and nobody moves; "
          "every robot sees that nobody moved -> e-stay", count_);
      } else {
        const bool one = leavers_.size() == 1;
        RCLCPP_INFO(
          get_logger(), "[bell] bell %d: %s %s faulty and %s for the calibration bay; every "
          "robot sees that someone left -> e-leave", count_, who.c_str(),
          one ? "knows it is" : "know they are", one ? "leaves" : "leave");
      }
    }
    const double held = (now() - since_).seconds();
    if (held > 2.0) {
      lamps_.put_out("bell_lamp");
    }
    // The leavers: from the ring as they go, the first of them through the
    // warehouse, then the bay as it arrives (the first takes about 35 s).
    if (!leavers_.empty() && held > 3.0 && shot_ == 0) {
      crew_.shot("muster");
      shot_ = 1;
    }
    if (!leavers_.empty() && held > 12.0 && shot_ == 1) {
      crew_.shot(leavers_.front());
      shot_ = 2;
    }
    if (!leavers_.empty() && held > 28.0 && shot_ == 2) {
      crew_.shot("bay");
      shot_ = 3;
    }
    if (held < dwell_) {
      send_feedback(0.4f, "the bell rings");
      return;
    }
    for (const auto & a : leavers_) {
      if (!crew_.arrived(a)) {
        send_feedback(0.8f, "leaving for the bay");
        return;
      }
    }
    ringing_ = false;
    shot_ = 0;
    crew_.finished();
    finish(true, 1.0, leavers_.empty() ? "nobody moved" : "the faulty left",
      leavers_.empty() ? "e-stay" : "e-leave");
  }

  Crew & crew_;
  Lamps & lamps_;
  Point bell_;
  double dwell_{7.0};
  bool ringing_{false};
  int shot_{0};
  bool waiting_{false};
  rclcpp::Time wait_from_;
  int count_{0};
  std::vector<std::string> leavers_;
  rclcpp::Time since_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr bell_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto side = std::make_shared<rclcpp::Node>("muster_crew");
  const auto lamp_dir = side->declare_parameter<std::string>("lamps", "/tmp/muddy_robots_lamps");
  Crew crew(side);
  Lamps lamps(side, lamp_dir);
  auto announce = std::make_shared<AnnounceAction>(crew, lamps, side);
  auto bell = std::make_shared<BellAction>(crew, lamps, side);
  // Each performer reads the action it performs from its own action_name
  // parameter, and the two share a process: they are set here and not by the
  // launch, whose parameters would reach both.
  announce->set_parameter(rclcpp::Parameter("action_name", "announce"));
  bell->set_parameter(rclcpp::Parameter("action_name", "bell"));
  announce->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  bell->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(side);
  executor.add_node(announce->get_node_base_interface());
  executor.add_node(bell->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
