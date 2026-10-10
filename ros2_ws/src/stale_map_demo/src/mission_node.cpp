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

// Runs one fleet through the shift, and judges it.
//
// Every fleet sees the same forklift on the same floor, and every fleet's
// haulers then drive to their drops by their own maps. What differs is how
// the fleet exchanges maps in between:
//
//   epistemic   the planner's policy over the radio floor: maps sent only
//               where a robot believes the recipient's map is stale and the
//               recipient needs it, merged by the time each cell was observed
//   fuse        every robot sends every other its map of every bay, merged
//               by epistemic_slam::fuse's rule, the more confident reading
//   overwrite   the same exchanges, the received reading taking the place
//               of the receiver's
//   recency     the same exchanges, merged by the time each cell was
//               observed, as the epistemic fleet merges
//
// On the secret floor r4 is a contractor's robot that hauls and must not be
// sent the staging of t1:
//
//   secret      the plan over the secret floor
//   flood       recency flooding with deltas, unlimited
//   flood3      the same, stopped after three maps of a bay, the plan's count
//   pull        each hauler asks every robot about each bay in turn
//
// whose sends are computed by study/protocols.py on this floor and arrive
// here as a sequence the mission runs in order.
//
// The fuse, overwrite and recency fleets send their maps in a fixed order, sender by sender, and
// then send the haulers with no bay named: each goes through whichever bay
// its map shows clear. The epistemic fleet runs its policy, in which the
// forklift's two changes come first and each hauler is sent through the bay
// it was planned to cross.
//
// What each fleet will do is known before it runs, from the domain and from
// the floor's claims (tools/fleets.py); a run that does something else is a
// failed run, whichever way it differs. A fleet that fails the goal the way
// the analysis says it fails is a run that worked.
//
// At the end the mission sets every robot's map against what the model says
// the robot believes, for the epistemic fleet, where there is a model.

#include <algorithm>
#include <chrono>
#include <fstream>
#include <map>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <nlohmann/json.hpp>

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_epistemic_msgs/srv/check_formula.hpp"
#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_msgs/action/execute_plan.hpp"
#include "plansys2_msgs/msg/plan.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;   // NOLINT(build/namespaces)
using nlohmann::json;

namespace
{

/// What each fleet does, from tools/fleets.py: the haulers that reach their
/// drops, and how many maps of a bay are sent.
struct Expected
{
  std::vector<std::string> delivered;
  std::size_t sends;
};

bool active(const rclcpp::Node::SharedPtr & node, const std::string & name, std::chrono::seconds patience)
{
  auto client = node->create_client<lifecycle_msgs::srv::GetState>(name + "/get_state");
  const auto deadline = std::chrono::steady_clock::now() + patience;
  while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
    if (client->wait_for_service(500ms)) {
      auto future = client->async_send_request(
        std::make_shared<lifecycle_msgs::srv::GetState::Request>());
      if (rclcpp::spin_until_future_complete(node, future, 2s) == rclcpp::FutureReturnCode::SUCCESS &&
        future.get()->current_state.id == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
      {
        return true;
      }
    }
    std::this_thread::sleep_for(500ms);
  }
  return false;
}

std::optional<bool> holds(const rclcpp::Node::SharedPtr & node, const std::string & formula)
{
  auto client = node->create_client<plansys2_epistemic_msgs::srv::CheckFormula>(
    "epistemic_state/check_formula");
  if (!client->wait_for_service(5s)) {return std::nullopt;}
  auto request = std::make_shared<plansys2_epistemic_msgs::srv::CheckFormula::Request>();
  request->formula = formula;
  auto future = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node, future, 5s) != rclcpp::FutureReturnCode::SUCCESS) {
    return std::nullopt;
  }
  const auto answer = future.get();
  if (!answer->success) {return std::nullopt;}
  return answer->holds;
}

class Floor
{
public:
  Floor(
    rclcpp::Node::SharedPtr node, const std::vector<std::string> & agents,
    const std::vector<std::string> & blocked_after,
    const std::vector<std::string> & contractors = {}, const std::vector<std::string> & secret = {})
  : node_(std::move(node)), blocked_after_(blocked_after), contractors_(contractors), secret_(secret)
  {
    const auto latched = rclcpp::QoS(1).transient_local().reliable();
    for (const auto & a : agents) {
      subs_.push_back(node_->create_subscription<std_msgs::msg::String>(
          "/" + a + "/readings", latched,
          [this, a](std_msgs::msg::String::SharedPtr m) {
            try {readings_[a] = json::parse(m->data);} catch (const std::exception &) {}
          }));
    }
    commands_ = node_->create_publisher<std_msgs::msg::String>(
      "/stale_maps/command", rclcpp::QoS(50).reliable());
    events_sub_ = node_->create_subscription<std_msgs::msg::String>(
      "/stale_maps/events", rclcpp::QoS(50).reliable(),
      [this](std_msgs::msg::String::SharedPtr m) {
        try {
          const auto e = json::parse(m->data);
          events_.push_back(e);
          if (e.value("verb", "") == "send") {
            ++sends_;
            // A write that made a correct reading stale: the receiver read
            // the bay as the floor has it before the merge, and not after.
            const auto bay = e["args"][2].get<std::string>();
            const bool blocked = std::find(
              blocked_after_.begin(), blocked_after_.end(), bay) != blocked_after_.end();
            const std::string truth = blocked ? "blocked" : "clear";
            if (e.value("before", "") == truth && e.value("after", "") != truth) {
              ++regressions_;
            }
            // A contractor now holds a secret change, told by a message.
            const auto to = e["args"][1].get<std::string>();
            if (std::find(contractors_.begin(), contractors_.end(), to) != contractors_.end() &&
              std::find(secret_.begin(), secret_.end(), bay) != secret_.end() &&
              e.value("after", "") == "blocked" && e.value("before", "") != "blocked")
            {
              leaks_.push_back(to + " " + bay + " from " + e["args"][0].get<std::string>());
            }
          }
          if (e.value("verb", "") == "haul" && e.value("ok", false)) {
            delivered_.push_back(e["args"][0].get<std::string>());
          }
        } catch (const std::exception &) {
        }
      });
  }

  bool reporting(std::size_t n) const {return readings_.size() >= n;}

  std::string verdict(const std::string & a, const std::string & bay) const
  {
    const auto it = readings_.find(a);
    if (it == readings_.end()) {return "";}
    return it->second["bays"][bay].value("verdict", "");
  }

  /// Send a command to the crew and wait for its event.
  json run(const std::string & verb, const std::vector<std::string> & args, std::chrono::seconds patience)
  {
    const int id = ++id_;
    json c = {{"id", id}, {"verb", verb}, {"args", args}};
    std_msgs::msg::String msg;
    msg.data = c.dump();
    commands_->publish(msg);
    const auto deadline = std::chrono::steady_clock::now() + patience;
    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
      rclcpp::spin_some(node_);
      for (const auto & e : events_) {
        if (e.value("id", -1) == id) {return e;}
      }
      std::this_thread::sleep_for(50ms);
    }
    return {{"ok", false}, {"message", "the crew did not answer " + verb}};
  }

  std::size_t sends() const {return sends_;}
  std::size_t regressions() const {return regressions_;}
  const std::vector<std::string> & leaks() const {return leaks_;}
  const std::vector<std::string> & delivered() const {return delivered_;}

private:
  rclcpp::Node::SharedPtr node_;
  std::vector<std::string> blocked_after_;
  std::vector<std::string> contractors_;
  std::vector<std::string> secret_;
  std::vector<std::string> leaks_;
  std::map<std::string, json> readings_;
  std::vector<rclcpp::Subscription<std_msgs::msg::String>::SharedPtr> subs_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr commands_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr events_sub_;
  std::vector<json> events_;
  std::vector<std::string> delivered_;
  std::size_t sends_{0};
  std::size_t regressions_{0};
  int id_{0};
};

int fail(const rclcpp::Node::SharedPtr & node, const std::string & why)
{
  RCLCPP_ERROR(node->get_logger(), "[mission] %s", why.c_str());
  rclcpp::shutdown();
  return 1;
}

/// The policy, as the executor will run it, for the log and the film.
void show(const rclcpp::Logger & log, const plansys2_msgs::msg::Plan & plan)
{
  for (std::size_t i = 0; i < plan.items.size(); ++i) {
    RCLCPP_INFO(log, "[policy]   %zu %s", i + 1, plan.items[i].epistemic_action.c_str());
  }
}

/// The epistemic fleet: plan, and run the plan.
bool run_policy(const rclcpp::Node::SharedPtr & node, const std::vector<std::string> & agents,
  const std::vector<std::string> & bays, const std::string & policy_out)
{
  auto problem = std::make_shared<plansys2::ProblemExpertClient>();
  auto planner = std::make_shared<plansys2::PlannerClient>();
  auto domain = std::make_shared<plansys2::DomainExpertClient>();
  auto executor = std::make_shared<plansys2::ExecutorClient>();

  for (const auto & name : {"domain_expert", "problem_expert", "epistemic_state", "planner", "executor"}) {
    if (!active(node, name, 240s)) {
      RCLCPP_ERROR(node->get_logger(), "[mission] %s never became active", name);
      return false;
    }
  }
  for (const auto & a : agents) {
    problem->addInstance(plansys2::Instance{a, "robot"});
    problem->addPredicate(plansys2::Predicate("(ready " + a + ")"));
  }
  for (const auto & b : bays) {
    problem->addInstance(plansys2::Instance{b, "bay"});
  }
  problem->addPredicate(plansys2::Predicate("(forklift_on_shift)"));
  problem->setGoal(plansys2::Goal("(and (shift_done))"));

  const auto started = node->now();
  const auto plan = planner->getPlan(domain->getDomain(), problem->getProblem());
  if (!plan.has_value()) {
    RCLCPP_ERROR(node->get_logger(), "[mission] no plan");
    return false;
  }
  RCLCPP_INFO(
    node->get_logger(), "[mission] policy with %zu nodes, in %.1f s", plan->items.size(),
    (node->now() - started).seconds());
  show(node->get_logger(), plan.value());
  if (!policy_out.empty()) {
    std::ofstream out(policy_out);
    json items = json::array();
    for (const auto & item : plan->items) {
      items.push_back({{"action", item.action}, {"epistemic_action", item.epistemic_action}});
    }
    out << json{{"goal", plan->epistemic_goal}, {"items", items}}.dump(2) << "\n";
  }

  if (!executor->start_plan_execution(plan.value())) {
    RCLCPP_ERROR(node->get_logger(), "[mission] the executor refused the policy");
    return false;
  }
  RCLCPP_INFO(node->get_logger(), "[mission] executing");
  rclcpp::Rate rate(4);
  while (rclcpp::ok() && executor->execute_and_check_plan()) {
    rclcpp::spin_some(node);
    rate.sleep();
  }
  const auto result = executor->getResult();
  return result && result->result == plansys2_msgs::action::ExecutePlan::Result::SUCCESS;
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("stale_maps_mission");
  const auto fleet = node->declare_parameter<std::string>("fleet", "epistemic");
  const auto agents = node->declare_parameter<std::vector<std::string>>("agents", std::vector<std::string>{});
  const auto haulers = node->declare_parameter<std::vector<std::string>>("haulers", std::vector<std::string>{});
  const auto bays = node->declare_parameter<std::vector<std::string>>("bays", std::vector<std::string>{});
  const auto after = node->declare_parameter<std::vector<std::string>>("blocked_after", std::vector<std::string>{});
  const auto changes = node->declare_parameter<std::vector<std::string>>("changes", std::vector<std::string>{});
  // A launch file cannot pass an empty list, so none is [""].
  auto expected_delivered = node->declare_parameter<std::vector<std::string>>(
    "expected_delivered", std::vector<std::string>{});
  expected_delivered.erase(
    std::remove(expected_delivered.begin(), expected_delivered.end(), std::string{}),
    expected_delivered.end());
  const auto expected_sends = node->declare_parameter<int>("expected_sends", 0);
  const auto expected_leak = node->declare_parameter<bool>("expected_leak", false);
  // A protocol's sends, computed by study/protocols.py on this floor and run
  // here in order: "sender receiver bay". Empty: every map of every bay to
  // every other robot, sender by sender. `requests` are the messages a pull
  // sends that carry no reading, counted with the rest.
  auto sequence = node->declare_parameter<std::vector<std::string>>("sequence", std::vector<std::string>{});
  sequence.erase(std::remove(sequence.begin(), sequence.end(), std::string{}), sequence.end());
  const auto requests = node->declare_parameter<int>("requests", 0);
  auto contractors = node->declare_parameter<std::vector<std::string>>("contractors", std::vector<std::string>{});
  contractors.erase(std::remove(contractors.begin(), contractors.end(), std::string{}), contractors.end());
  auto secret = node->declare_parameter<std::vector<std::string>>("secret", std::vector<std::string>{});
  secret.erase(std::remove(secret.begin(), secret.end(), std::string{}), secret.end());
  const bool planned = fleet == "epistemic" || fleet == "secret";
  const auto policy_out = node->declare_parameter<std::string>("policy_out", "");
  const double hold = node->declare_parameter<double>("hold", 0.0);

  Floor floor(node, agents, after, contractors, secret);
  RCLCPP_INFO(node->get_logger(), "[mission] fleet %s: %zu robots, %zu haulers", fleet.c_str(),
    agents.size(), haulers.size());

  // Every map up and reporting before anything happens on the floor.
  const auto deadline = std::chrono::steady_clock::now() + 240s;
  while (rclcpp::ok() && !floor.reporting(agents.size())) {
    if (std::chrono::steady_clock::now() > deadline) {
      return fail(node, "not every robot's map reported");
    }
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(200ms);
  }
  std::string start;
  for (const auto & a : agents) {
    start += " " + a + ":" + floor.verdict(a, "t1").substr(0, 1) + floor.verdict(a, "t2").substr(0, 1) +
      floor.verdict(a, "t3").substr(0, 1);
  }
  RCLCPP_INFO(node->get_logger(), "[mission] every map is the shift map:%s", start.c_str());
  if (hold > 0.0) {
    RCLCPP_INFO(node->get_logger(), "[mission] holding %.0f s", hold);
    rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(hold * 1000)));
  }

  bool ran = true;
  if (planned) {
    ran = run_policy(node, agents, bays, policy_out);
    if (!ran) {
      RCLCPP_ERROR(node->get_logger(), "[mission] the policy did not complete");
    }
  } else {
    // The forklift, in the order the policy has it.
    for (const auto & change : changes) {
      const auto verb = change.substr(0, change.find(' '));
      const auto bay = change.substr(change.find(' ') + 1);
      const auto e = floor.run(verb, {bay}, 120s);
      if (!e.value("ok", false)) {
        return fail(node, "the forklift: " + e.value("message", std::string{}));
      }
    }
    if (!sequence.empty()) {
      RCLCPP_INFO(node->get_logger(), "[mission] %s: %zu maps of a bay sent as the protocol sends them, "
        "and %ld requests", fleet.c_str(), sequence.size(), static_cast<long>(requests));
      for (const auto & line : sequence) {
        std::istringstream in(line);
        std::string from, to, bay;
        in >> from >> to >> bay;
        const auto e = floor.run("send", {from, to, bay}, 30s);
        if (!e.value("ok", false)) {
          return fail(node, "an exchange failed: " + e.value("message", std::string{}));
        }
      }
    } else {
      // Every map of every bay to every other robot.
      RCLCPP_INFO(node->get_logger(), "[mission] %s: every robot sends every other its map of every bay",
        fleet.c_str());
      for (const auto & from : agents) {
        for (const auto & to : agents) {
          if (from == to) {continue;}
          for (const auto & bay : bays) {
            const auto e = floor.run("send", {from, to, bay}, 30s);
            if (!e.value("ok", false)) {
              return fail(node, "an exchange failed: " + e.value("message", std::string{}));
            }
          }
        }
      }
    }
    std::string maps;
    for (const auto & a : agents) {
      maps += " " + a + ":" + floor.verdict(a, "t1").substr(0, 1) + floor.verdict(a, "t2").substr(0, 1) +
        floor.verdict(a, "t3").substr(0, 1);
    }
    RCLCPP_INFO(node->get_logger(), "[mission] after %zu maps sent:%s", floor.sends() + requests,
      maps.c_str());
    for (const auto & h : haulers) {
      floor.run("haul", {h}, 420s);
    }
  }

  // ─── The verdict ──────────────────────────────────────────────────────────
  for (int i = 0; i < 10; ++i) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(100ms);
  }
  std::size_t stale = 0;
  std::string rows;
  for (const auto & a : agents) {
    rows += " " + a + ":";
    for (const auto & bay : bays) {
      const auto v = floor.verdict(a, bay);
      const bool blocked = std::find(after.begin(), after.end(), bay) != after.end();
      const bool wrong = (v == "clear" && blocked) || (v == "blocked" && !blocked);
      stale += wrong ? 1 : 0;
      rows += v.substr(0, 1) + (wrong ? "*" : "");
    }
  }
  const auto & delivered = floor.delivered();
  std::string got;
  for (const auto & h : delivered) {got += " " + h;}
  RCLCPP_INFO(
    node->get_logger(), "[mission] verdict: %zu of %zu haulers delivered (%s ); %zu maps of a bay "
    "sent, %zu of them over a fresh reading; %zu stale entries at the end:%s", delivered.size(),
    haulers.size(), got.empty() ? " none" : got.c_str(), floor.sends() + requests, floor.regressions(), stale,
    rows.c_str());
  if (!contractors.empty()) {
    std::string leaked;
    for (const auto & l : floor.leaks()) {leaked += " " + l + ";";}
    RCLCPP_INFO(node->get_logger(), "[mission] secret: %s", floor.leaks().empty() ?
      "no contractor was sent it" : ("sent to a contractor:" + leaked).c_str());
  }

  if (planned) {
    // The model against the maps: B_i blocked(t), B_i not blocked(t), or
    // neither, for every robot and bay.
    std::size_t agree = 0, differ = 0;
    std::string differences;
    for (const auto & a : agents) {
      for (const auto & bay : bays) {
        const auto b = holds(node, "(K " + a + " blocked_" + bay + ")");
        const auto o = holds(node, "(K " + a + " (not blocked_" + bay + "))");
        if (!b || !o) {return fail(node, "the epistemic state could not answer for " + a);}
        const std::string model = *b ? "blocked" : *o ? "clear" : "unknown";
        const auto map = floor.verdict(a, bay);
        if (model == map) {
          ++agree;
        } else {
          ++differ;
          differences += " " + a + " " + bay + ": model " + model + ", map " + map + ";";
        }
      }
    }
    RCLCPP_INFO(
      node->get_logger(), "[mission] the model and the maps agree on %zu of %zu beliefs%s%s",
      agree, agree + differ, differ ? "; they differ on" : "", differences.c_str());
  }

  std::vector<std::string> sorted_got = delivered, sorted_want = expected_delivered;
  std::sort(sorted_got.begin(), sorted_got.end());
  std::sort(sorted_want.begin(), sorted_want.end());
  const bool as_expected = sorted_got == sorted_want &&
    floor.sends() + requests == static_cast<std::size_t>(expected_sends) && (!planned || ran) &&
    floor.leaks().empty() != expected_leak;
  if (!as_expected) {
    std::string want;
    for (const auto & h : expected_delivered) {want += " " + h;}
    return fail(node, "the run did not do what the analysis says " + fleet + " does: expected" +
      (want.empty() ? " none" : want) + " delivered and " + std::to_string(expected_sends) +
      " maps sent");
  }
  RCLCPP_INFO(node->get_logger(), "[mission] the run did what the analysis says %s does",
    fleet.c_str());
  rclcpp::shutdown();
  return 0;
}
