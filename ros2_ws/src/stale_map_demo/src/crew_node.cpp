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

// What happens on the floor, for every fleet.
//
// Four things can happen, and each is a command on /stale_maps/command and
// an event on /stale_maps/events, both JSON:
//
//   stage t           the forklift puts a load in bay t
//   clear t           the forklift takes the load out of bay t
//   send i j t [v]    robot i's map of bay t is sent to robot j and merged by
//                     j's rule; with v, j's map must then read t as v
//   haul h [t]        hauler h drives to its drop by its own map, through t
//                     when t is given and through any bay its map shows clear
//                     when it is not
//
// The epistemic fleet reaches these through the plan: each performer relays
// its action here and waits for the event. The other fleets' missions send
// the same commands themselves. Either way the floor is the same floor and
// the robots the same robots, so what differs between two fleets' runs is
// what they decided, and nothing in how it was carried out.
//
// The forklift's two changes are checked against the floor's claim: after
// a change every robot the claim says sees the bay must read the change on
// its own map, and every other robot must read the bay as it did before.
// That is the observability of the domain's change action, measured.

#include <chrono>
#include <cmath>
#include <algorithm>
#include <fstream>
#include <future>
#include <map>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "epistemic_msgs/srv/offer_map.hpp"
#include "epistemic_msgs/srv/receive_map.hpp"
#include "gazebo_msgs/srv/delete_entity.hpp"
#include "gazebo_msgs/srv/spawn_entity.hpp"
#include "pass_through_demo/driver.hpp"
#include "pass_through_demo/grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;   // NOLINT(build/namespaces)
using nlohmann::json;

namespace
{

std::string read_file(const std::string & path)
{
  std::ifstream in(path);
  if (!in) {throw std::runtime_error("cannot read " + path);}
  return {std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
}

std::vector<std::string> words(const std::string & text)
{
  std::istringstream in(text);
  std::vector<std::string> out;
  for (std::string w; in >> w; ) {out.push_back(w);}
  return out;
}

template<typename F>
bool ready(const std::shared_future<F> & f)
{
  return f.valid() && f.wait_for(0s) == std::future_status::ready;
}

}  // namespace

class Crew : public rclcpp::Node
{
public:
  Crew()
  : rclcpp::Node("stale_maps_crew")
  {
    agents_ = declare_parameter<std::vector<std::string>>("agents", std::vector<std::string>{});
    const auto haulers = declare_parameter<std::vector<std::string>>("haulers", std::vector<std::string>{});
    const auto drops = declare_parameter<std::vector<double>>("drops", std::vector<double>{});
    drop_radius_ = declare_parameter<double>("drop_radius", 0.5);
    floorplan_ = declare_parameter<std::string>("floorplan", "");
    const auto bay_names = declare_parameter<std::vector<std::string>>("bays", std::vector<std::string>{});
    const auto bay_boxes = declare_parameter<std::vector<double>>("bay_boxes", std::vector<double>{});
    const auto centres = declare_parameter<std::vector<double>>("bay_centres", std::vector<double>{});
    load_sdf_ = declare_parameter<std::string>("load_sdf", "");
    const auto obs_bays = declare_parameter<std::vector<std::string>>("observer_bays", std::vector<std::string>{});
    const auto obs_lists = declare_parameter<std::vector<std::string>>("observer_lists", std::vector<std::string>{});
    settle_ = declare_parameter<double>("settle_seconds", 30.0);
    transfer_ = declare_parameter<double>("transfer_seconds", 3.0);
    haul_timeout_ = declare_parameter<double>("haul_timeout", 360.0);

    bays_ = pass_through::boxes_from(bay_names, bay_boxes);
    for (std::size_t k = 0; k < bay_names.size() && 2 * k + 1 < centres.size(); ++k) {
      centres_[bay_names[k]] = {centres[2 * k], centres[2 * k + 1]};
    }
    for (std::size_t k = 0; k < haulers.size() && 2 * k + 1 < drops.size(); ++k) {
      drops_[haulers[k]] = {drops[2 * k], drops[2 * k + 1]};
    }
    for (std::size_t k = 0; k < obs_bays.size() && k < obs_lists.size(); ++k) {
      observers_[obs_bays[k]] = words(obs_lists[k]);
    }

    const auto latched = rclcpp::QoS(1).transient_local().reliable();
    for (const auto & a : agents_) {
      readings_subs_.push_back(create_subscription<std_msgs::msg::String>(
          "/" + a + "/readings", latched,
          [this, a](std_msgs::msg::String::SharedPtr m) {
            try {readings_[a] = json::parse(m->data);} catch (const std::exception &) {}
          }));
      offer_[a] = create_client<epistemic_msgs::srv::OfferMap>("/" + a + "/offer");
      receive_[a] = create_client<epistemic_msgs::srv::ReceiveMap>("/" + a + "/receive");
    }
    spawn_ = create_client<gazebo_msgs::srv::SpawnEntity>("/spawn_entity");
    delete_ = create_client<gazebo_msgs::srv::DeleteEntity>("/delete_entity");

    events_ = create_publisher<std_msgs::msg::String>("/stale_maps/events", rclcpp::QoS(50).reliable());
    link_ = create_publisher<std_msgs::msg::String>("/stale_maps/link", latched);
    acting_ = create_publisher<std_msgs::msg::String>("/stale_maps/acting", latched);
    commands_ = create_subscription<std_msgs::msg::String>(
      "/stale_maps/command", rclcpp::QoS(50).reliable(),
      [this](std_msgs::msg::String::SharedPtr m) {on_command(m->data);});

    timer_ = create_wall_timer(100ms, [this]() {step();});
  }

  /// The drivers need a shared pointer to this node, which the constructor
  /// cannot hand out.
  void start()
  {
    for (const auto & [h, drop] : drops_) {
      (void)drop;
      pass_through::Driver::Config c;
      c.agent = h;
      c.ns = h;
      c.floorplan = floorplan_;
      c.bays = bays_;
      for (const auto & a : agents_) {
        if (a != h) {c.others.push_back(a);}
      }
      drivers_[h] = std::make_unique<pass_through::Driver>(shared_from_this(), c);
    }
    RCLCPP_INFO(
      get_logger(), "[crew] %zu robots, %zu of them hauling; %zu bays", agents_.size(),
      drivers_.size(), bays_.size());
  }

private:
  struct Job
  {
    int id{0};
    std::string verb;
    std::vector<std::string> args;
    int phase{0};
    rclcpp::Time since;
    json details;
    std::map<std::string, std::string> before;
    std::shared_future<gazebo_msgs::srv::SpawnEntity::Response::SharedPtr> spawned;
    std::shared_future<gazebo_msgs::srv::DeleteEntity::Response::SharedPtr> deleted;
    std::shared_future<epistemic_msgs::srv::OfferMap::Response::SharedPtr> offered;
    std::shared_future<epistemic_msgs::srv::ReceiveMap::Response::SharedPtr> received;
    std::optional<rclcpp::Time> no_route_since;
    std::vector<std::string> crossed;
    double driven{0.0};
    double last_x{0.0}, last_y{0.0};
    bool have_last{false};
  };

  // ─── Plumbing ─────────────────────────────────────────────────────────────

  void on_command(const std::string & text)
  {
    try {
      const auto c = json::parse(text);
      Job job;
      job.id = c.value("id", 0);
      job.verb = c.value("verb", "");
      job.args = c.value("args", std::vector<std::string>{});
      queue_.push_back(std::move(job));
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "[crew] unreadable command %s: %s", text.c_str(), e.what());
    }
  }

  void emit(Job & job, bool ok, const std::string & message)
  {
    json e = job.details;
    e["id"] = job.id;
    e["verb"] = job.verb;
    e["args"] = job.args;
    e["ok"] = ok;
    e["message"] = message;
    e["seconds"] = (now() - job.since).seconds();
    std_msgs::msg::String msg;
    msg.data = e.dump();
    events_->publish(msg);
    if (ok) {
      RCLCPP_INFO(get_logger(), "[crew] done: %s", message.c_str());
    } else {
      RCLCPP_ERROR(get_logger(), "[crew] failed: %s", message.c_str());
    }
    current_.reset();
  }

  void say(rclcpp::Publisher<std_msgs::msg::String>::SharedPtr & pub, const std::string & text)
  {
    std_msgs::msg::String m;
    m.data = text;
    pub->publish(m);
  }

  std::string verdict(const std::string & agent, const std::string & bay) const
  {
    const auto it = readings_.find(agent);
    if (it == readings_.end() || !it->second.contains("bays") || !it->second["bays"].contains(bay)) {
      return "";
    }
    return it->second["bays"][bay].value("verdict", "");
  }

  json map_row(const std::string & agent) const
  {
    json row;
    for (const auto & [bay, box] : bays_) {
      (void)box;
      row[bay] = verdict(agent, bay);
    }
    return row;
  }

  void step()
  {
    if (!current_ && !queue_.empty()) {
      current_ = std::move(queue_.front());
      queue_.erase(queue_.begin());
      current_->since = now();
      std::string text;
      for (const auto & a : current_->args) {text += " " + a;}
      RCLCPP_INFO(get_logger(), "[crew] %s%s", current_->verb.c_str(), text.c_str());
    }
    if (!current_) {return;}
    Job & job = *current_;
    if (job.verb == "stage" || job.verb == "clear") {
      forklift(job);
    } else if (job.verb == "send") {
      send(job);
    } else if (job.verb == "haul") {
      haul(job);
    } else {
      emit(job, false, "no such command: " + job.verb);
    }
  }

  // ─── The forklift ─────────────────────────────────────────────────────────

  void forklift(Job & job)
  {
    if (job.args.empty() || !bays_.count(job.args[0])) {
      emit(job, false, job.verb + " needs a bay");
      return;
    }
    const auto & bay = job.args[0];
    const bool stage = job.verb == "stage";
    const std::string expect = stage ? "blocked" : "clear";
    const auto observers = observers_.count(bay) ? observers_.at(bay) : std::vector<std::string>{};

    if (job.phase == 0) {
      for (const auto & a : agents_) {
        job.before[a] = verdict(a, bay);
        if (job.before[a].empty()) {
          return;   // a map that has not reported yet
        }
      }
      say(acting_, "forklift " + bay);
      if (stage) {
        if (!spawn_->service_is_ready()) {return;}
        auto req = std::make_shared<gazebo_msgs::srv::SpawnEntity::Request>();
        req->name = "load_" + bay;
        req->xml = read_file(load_sdf_);
        req->initial_pose.position.x = centres_.at(bay).first;
        req->initial_pose.position.y = centres_.at(bay).second;
        req->reference_frame = "world";
        job.spawned = spawn_->async_send_request(req).share();
      } else {
        if (!delete_->service_is_ready()) {return;}
        auto req = std::make_shared<gazebo_msgs::srv::DeleteEntity::Request>();
        req->name = "load_" + bay;
        job.deleted = delete_->async_send_request(req).share();
      }
      RCLCPP_INFO(
        get_logger(), "[forklift] %s %s; the floor says %zu robots see it", stage ? "stages a load in" :
        "takes the load out of", bay.c_str(), observers.size());
      job.phase = 1;
      return;
    }
    if (job.phase == 1) {
      if (stage ? !ready(job.spawned) : !ready(job.deleted)) {return;}
      const bool done = stage ? job.spawned.get()->success : job.deleted.get()->success;
      if (!done) {
        emit(job, false, "Gazebo refused to " + job.verb + " " + bay);
        return;
      }
      job.since = now();
      job.phase = 2;
      return;
    }

    // Wait for every observer's map to read the change, and a little longer,
    // so a map that was not meant to change has had time to.
    bool all = true;
    for (const auto & a : observers) {all = all && verdict(a, bay) == expect;}
    const double waited = (now() - job.since).seconds();
    if (job.phase == 2) {
      if (!all && waited < settle_) {return;}
      job.phase = 3;
      job.details["read_after"] = waited;
      job.since = now();
      return;
    }
    if ((now() - job.since).seconds() < 2.0) {return;}

    json table;
    std::vector<std::string> wrong;
    for (const auto & a : agents_) {
      const bool sees = std::find(observers.begin(), observers.end(), a) != observers.end();
      const auto v = verdict(a, bay);
      table[a] = {{"before", job.before[a]}, {"after", v}, {"sees", sees}};
      if ((sees && v != expect) || (!sees && v != job.before[a])) {wrong.push_back(a);}
    }
    job.details["bay"] = bay;
    job.details["robots"] = table;
    std::string seen;
    for (const auto & a : observers) {seen += " " + a;}
    if (!wrong.empty()) {
      std::string who;
      for (const auto & a : wrong) {who += " " + a + " reads " + verdict(a, bay) + ";";}
      emit(job, false, "the maps after " + job.verb + " " + bay + " disagree with the floor:" + who);
      return;
    }
    std::ostringstream msg;
    msg << job.verb << " " << bay << ": read " << expect << " by" << seen << " after "
        << job.details["read_after"].get<double>() << " s; every other map unchanged";
    emit(job, true, msg.str());
  }

  // ─── A map sent ───────────────────────────────────────────────────────────

  void send(Job & job)
  {
    if (job.args.size() < 3) {
      emit(job, false, "send needs a sender, a receiver and a bay");
      return;
    }
    const auto & from = job.args[0];
    const auto & to = job.args[1];
    const auto & bay = job.args[2];
    const std::string expect = job.args.size() > 3 ? job.args[3] : "";
    if (!offer_.count(from) || !receive_.count(to)) {
      emit(job, false, "send names a robot the crew does not know");
      return;
    }
    if (job.phase == 0) {
      if (!offer_[from]->service_is_ready() || !receive_[to]->service_is_ready()) {return;}
      auto req = std::make_shared<epistemic_msgs::srv::OfferMap::Request>();
      req->region = bay;
      job.offered = offer_[from]->async_send_request(req).share();
      job.phase = 1;
      return;
    }
    if (job.phase == 1) {
      if (!ready(job.offered)) {return;}
      const auto offered = job.offered.get();
      if (!offered->ok) {
        emit(job, false, offered->message);
        return;
      }
      job.details["sender_reads"] = offered->verdict;
      auto req = std::make_shared<epistemic_msgs::srv::ReceiveMap::Request>();
      req->source = from;
      req->region = bay;
      req->map = offered->map;
      req->observed = offered->observed;
      job.received = receive_[to]->async_send_request(req).share();
      say(link_, from + " " + to + " " + bay + " " + offered->verdict);
      job.phase = 2;
      return;
    }
    if (job.phase == 2) {
      if (!ready(job.received)) {return;}
      const auto r = job.received.get();
      job.details["rule"] = r->rule;
      job.details["learned"] = r->learned;
      job.details["changed"] = r->changed;
      job.details["kept"] = r->kept;
      job.details["before"] = r->verdict_before;
      job.details["after"] = r->verdict_after;
      if (!r->ok) {
        say(link_, "");
        emit(job, false, r->message);
        return;
      }
      job.phase = 3;
      return;
    }
    // Held long enough for the link to be seen, when it is to be seen.
    if ((now() - job.since).seconds() < transfer_) {return;}
    say(link_, "");
    const auto after = job.details["after"].get<std::string>();
    std::ostringstream msg;
    msg << from << " sent " << to << " its map of " << bay << " (it reads "
        << job.details["sender_reads"].get<std::string>() << "); merged by "
        << job.details["rule"].get<std::string>() << ": " << job.details["changed"].get<unsigned>()
        << " cells replaced, " << job.details["kept"].get<unsigned>() << " kept against it; "
        << to << " read " << job.details["before"].get<std::string>() << ", now " << after;
    if (!expect.empty() && after != expect) {
      emit(job, false, msg.str() + ", and the plan said " + expect);
      return;
    }
    emit(job, true, msg.str());
  }

  // ─── A haul ───────────────────────────────────────────────────────────────

  void haul(Job & job)
  {
    if (job.args.empty() || !drivers_.count(job.args[0])) {
      emit(job, false, "haul needs a hauler");
      return;
    }
    const auto & h = job.args[0];
    const std::string through = job.args.size() > 1 ? job.args[1] : "";
    auto & driver = *drivers_.at(h);
    const auto [dx, dy] = drops_.at(h);

    if (job.phase == 0) {
      double x, y, yaw;
      if (!driver.pose(x, y, yaw) || verdict(h, bays_.begin()->first).empty()) {return;}
      std::vector<pass_through::Box> closed;
      if (!through.empty()) {
        for (const auto & [bay, box] : bays_) {
          if (bay != through) {closed.push_back(box);}
        }
      }
      driver.set_closed(closed);
      const auto a = driver.assess(dx, dy, drop_radius_, true);
      std::string route;
      for (const auto & b : a.route_bays) {route += " " + b;}
      job.details["map_at_start"] = map_row(h);
      job.details["planned_through"] = a.route_bays;
      job.details["planned_length"] = a.route_length;
      RCLCPP_INFO(
        get_logger(), "[haul] %s, by its map (t1 %s, t2 %s, t3 %s)%s: %s", h.c_str(),
        verdict(h, "t1").c_str(), verdict(h, "t2").c_str(), verdict(h, "t3").c_str(),
        through.empty() ? "" : (", through " + through).c_str(),
        a.reachable ? ("a route of " + std::to_string(a.route_length).substr(0, 5) +
        " m through" + route).c_str() : "no route");
      say(acting_, h);
      job.phase = 1;
      return;
    }

    double x, y, yaw;
    if (driver.pose(x, y, yaw)) {
      if (job.have_last) {job.driven += std::hypot(x - job.last_x, y - job.last_y);}
      job.last_x = x;
      job.last_y = y;
      job.have_last = true;
      for (const auto & [bay, box] : bays_) {
        if (box.contains(x, y) &&
          std::find(job.crossed.begin(), job.crossed.end(), bay) == job.crossed.end())
        {
          job.crossed.push_back(bay);
          RCLCPP_INFO(get_logger(), "[haul] %s enters %s", h.c_str(), bay.c_str());
        }
      }
    }
    job.details["entered"] = job.crossed;
    job.details["driven"] = job.driven;

    const auto progress = driver.step_to(dx, dy, drop_radius_);
    const auto finish = [&](bool ok, const std::string & why) {
        driver.stop();
        driver.set_closed({});
        job.details["map_at_end"] = map_row(h);
        std::string entered;
        for (const auto & b : job.crossed) {entered += " " + b;}
        std::ostringstream msg;
        msg << h << " " << why << " after " << static_cast<int>((now() - job.since).seconds())
            << " s, " << static_cast<int>(job.driven * 10) / 10.0 << " m driven; entered"
            << (entered.empty() ? " no bay" : entered) << "; its map reads t1 " << verdict(h, "t1")
            << ", t2 " << verdict(h, "t2") << ", t3 " << verdict(h, "t3");
        emit(job, ok, msg.str());
      };

    if (progress == pass_through::Driver::Progress::Arrived) {
      finish(true, "delivered");
      return;
    }
    if (progress == pass_through::Driver::Progress::NoRoute) {
      // A route lost for a moment while a map is being rewritten is not a
      // hauler with no route; four seconds of it is.
      if (!job.no_route_since) {job.no_route_since = now();}
      if ((now() - *job.no_route_since).seconds() > 4.0) {
        finish(false, "has no route on its map");
      }
      return;
    }
    job.no_route_since.reset();
    if ((now() - job.since).seconds() > haul_timeout_) {
      finish(false, "did not arrive");
    }
  }

  std::vector<std::string> agents_;
  std::map<std::string, std::pair<double, double>> drops_;
  std::map<std::string, std::pair<double, double>> centres_;
  std::map<std::string, pass_through::Box> bays_;
  std::map<std::string, std::vector<std::string>> observers_;
  std::string floorplan_;
  std::string load_sdf_;
  double drop_radius_{0.5};
  double settle_{30.0};
  double transfer_{3.0};
  double haul_timeout_{360.0};

  std::map<std::string, json> readings_;
  std::map<std::string, std::unique_ptr<pass_through::Driver>> drivers_;
  std::vector<Job> queue_;
  std::optional<Job> current_;

  std::vector<rclcpp::Subscription<std_msgs::msg::String>::SharedPtr> readings_subs_;
  std::map<std::string, rclcpp::Client<epistemic_msgs::srv::OfferMap>::SharedPtr> offer_;
  std::map<std::string, rclcpp::Client<epistemic_msgs::srv::ReceiveMap>::SharedPtr> receive_;
  rclcpp::Client<gazebo_msgs::srv::SpawnEntity>::SharedPtr spawn_;
  rclcpp::Client<gazebo_msgs::srv::DeleteEntity>::SharedPtr delete_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr events_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr link_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr acting_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr commands_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto crew = std::make_shared<Crew>();
  crew->start();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(crew);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
