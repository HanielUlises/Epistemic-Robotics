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

// Runs one fleet through the hotel's distributed leak.
//
// The robots first go where the model starts them: the concierge in the lobby
// with the guest, the porter in the kitchen, the cleaner in the restaurant.
// The mission then asks the planner for the fleet's policy and runs it. Which
// fleet it is decides which problem the planner was given, and so which
// channels the robots may use and which goal the policy is for.
//
// Whatever the fleet planned for, it is judged by the whole goal, at the
// actual world, by the knowledge view: the leak contained, every responder
// knowing that every responder knows it, and the guest unable to rule out any
// room. Each fleet's verdict is known before it runs, from the planner and
// tools/trace.py, and the mission completes only if the run gives that
// verdict. A baseline that fails the goal the way the analysis says it fails
// is a run that worked.
//
// Last, the cleaner and the concierge each do what they know: stand down if
// they know the leak is contained, stay where they are if they do not.

#include <cctype>
#include <chrono>
#include <fstream>
#include <map>
#include <memory>
#include <optional>
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
using plansys2_msgs::msg::PlanItem;

namespace
{

struct Verdict
{
  bool safe;
  bool stand_down;
  bool secret;
};

// What each fleet's policy does to the whole goal, from the planner's
// policies replayed by tools/trace.py. A run that disagrees is a failed run.
const std::map<std::string, Verdict> EXPECTED = {
  {"epistemic", {true, true, true}},
  {"broadcast", {true, true, false}},
  {"filter", {true, true, false}},
  {"siloed", {true, false, true}},
};

bool active(
  const rclcpp::Node::SharedPtr & node, const std::string & name,
  std::chrono::seconds patience)
{
  auto client = node->create_client<lifecycle_msgs::srv::GetState>(name + "/get_state");
  const auto deadline = std::chrono::steady_clock::now() + patience;
  while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
    if (client->wait_for_service(500ms)) {
      auto future = client->async_send_request(
        std::make_shared<lifecycle_msgs::srv::GetState::Request>());
      if (rclcpp::spin_until_future_complete(node, future, 2s) ==
        rclcpp::FutureReturnCode::SUCCESS &&
        future.get()->current_state.id == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
      {
        return true;
      }
    }
    std::this_thread::sleep_for(500ms);
  }
  return false;
}

void show(
  const rclcpp::Logger & log, const plansys2_msgs::msg::Plan & plan, std::size_t index,
  const std::string & prefix, const std::string & label)
{
  if (index >= plan.items.size()) {
    return;
  }
  const auto & item = plan.items[index];
  RCLCPP_INFO(
    log, "[policy] %s%s%s", prefix.c_str(), label.c_str(), item.epistemic_action.c_str());
  for (std::size_t i = 0; i < item.children.size(); ++i) {
    const auto child = item.children[i];
    const std::string outcome = i < item.outcomes.size() ? item.outcomes[i] : std::string{"?"};
    if (child == PlanItem::POLICY_DONE) {
      RCLCPP_INFO(log, "[policy] %s  %s -> done", prefix.c_str(), outcome.c_str());
      continue;
    }
    show(
      log, plan, child, prefix + "  ",
      item.children.size() > 1 ? outcome + " -> " : std::string{});
  }
}

void write_policy(const std::string & path, const plansys2_msgs::msg::Plan & plan)
{
  nlohmann::json out;
  out["goal"] = plan.epistemic_goal;
  out["items"] = nlohmann::json::array();
  for (std::size_t i = 0; i < plan.items.size(); ++i) {
    const auto & item = plan.items[i];
    nlohmann::json children = nlohmann::json::array();
    for (const auto c : item.children) {
      children.push_back(c == PlanItem::POLICY_DONE ? -1 : static_cast<int>(c));
    }
    out["items"].push_back(
      {{"index", i}, {"action", item.action}, {"epistemic_action", item.epistemic_action},
        {"sensing", item.sensing}, {"children", children}, {"outcomes", item.outcomes}});
  }
  std::ofstream(path) << out.dump(2) << "\n";
}

std::optional<bool> holds(const rclcpp::Node::SharedPtr & node, const std::string & formula)
{
  auto client = node->create_client<plansys2_epistemic_msgs::srv::CheckFormula>(
    "epistemic_state/check_formula");
  if (!client->wait_for_service(5s)) {
    return std::nullopt;
  }
  auto request = std::make_shared<plansys2_epistemic_msgs::srv::CheckFormula::Request>();
  request->formula = formula;
  auto future = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node, future, 5s) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    return std::nullopt;
  }
  const auto answer = future.get();
  if (!answer->success) {
    return std::nullopt;
  }
  return answer->holds;
}

/// The latest message on a latched topic that starts with @p prefix, or empty.
std::string await_message(
  const rclcpp::Node::SharedPtr & node, const std::string & topic,
  const std::vector<std::string> & prefixes, std::chrono::seconds patience)
{
  std::string heard;
  auto sub = node->create_subscription<std_msgs::msg::String>(
    topic, rclcpp::QoS(10).transient_local().reliable(),
    [&](std_msgs::msg::String::SharedPtr m) {
      for (const auto & p : prefixes) {
        if (m->data.rfind(p, 0) == 0) {
          heard = m->data;
        }
      }
    });
  const auto deadline = std::chrono::steady_clock::now() + patience;
  while (rclcpp::ok() && heard.empty() && std::chrono::steady_clock::now() < deadline) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(100ms);
  }
  return heard;
}

const char * yes(bool b)
{
  return b ? "yes" : "no";
}

int fail(const rclcpp::Node::SharedPtr & node, const std::string & why)
{
  RCLCPP_ERROR(node->get_logger(), "[mission] mission failed: %s", why.c_str());
  rclcpp::shutdown();
  return 1;
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("hotel_distributed_mission");
  const auto fleet = node->declare_parameter<std::string>("fleet", "epistemic");
  const auto leak = node->declare_parameter<std::string>("leak", "L3_room1");
  const auto policy_out = node->declare_parameter<std::string>("policy_out", "");
  const double hold = node->declare_parameter<double>("hold", 0.0);

  if (EXPECTED.find(fleet) == EXPECTED.end()) {
    return fail(node, "no fleet named " + fleet);
  }
  const auto expected = EXPECTED.at(fleet);

  auto request_pub = node->create_publisher<std_msgs::msg::String>(
    "/hotel/crew/request", rclcpp::QoS(10).reliable());
  const auto ask = [&](const std::string & what) {
      std_msgs::msg::String m;
      m.data = what;
      request_pub->publish(m);
    };

  auto problem = std::make_shared<plansys2::ProblemExpertClient>();
  auto planner = std::make_shared<plansys2::PlannerClient>();
  auto domain = std::make_shared<plansys2::DomainExpertClient>();
  auto executor = std::make_shared<plansys2::ExecutorClient>();

  RCLCPP_INFO(node->get_logger(), "waiting for the planning system");
  for (const auto & name :
    {"domain_expert", "problem_expert", "epistemic_state", "planner", "executor"})
  {
    if (!active(node, name, 240s)) {
      return fail(node, std::string(name) + " never became active");
    }
  }

  if (hold > 0.0) {
    RCLCPP_INFO(node->get_logger(), "[mission] holding %.0f s", hold);
    rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(hold * 1000)));
  }

  // The scene the model starts in.
  while (rclcpp::ok() && request_pub->get_subscription_count() == 0) {
    rclcpp::sleep_for(200ms);
  }
  RCLCPP_INFO(
    node->get_logger(), "[mission] the %s fleet; the leak is in %s, which no robot knows",
    fleet.c_str(), leak.c_str());
  ask("setup");
  const auto ready = await_message(node, "/hotel/crew", {"ready", "setup failed"}, 1200s);
  if (ready.rfind("ready", 0) != 0) {
    return fail(node, ready.empty() ? "the robots never reached their places" : ready);
  }
  RCLCPP_INFO(node->get_logger(), "[mission] %s", ready.c_str());

  // The classical half: where the robots are, in the executor's vocabulary.
  for (const auto & z : {"lobby", "restaurant", "kitchen", "clean_restaurant", "clean_lobby",
      "l2_room1", "l2_room15", "l3_room1", "l3_room15"})
  {
    problem->addInstance(plansys2::Instance{z, "zone"});
  }
  for (const auto & r : {"cleaner", "concierge", "porter"}) {
    problem->addInstance(plansys2::Instance{r, "robot"});
  }
  bool asserted = true;
  asserted &= problem->addPredicate(plansys2::Predicate("(robot_at concierge lobby)"));
  asserted &= problem->addPredicate(plansys2::Predicate("(robot_at porter kitchen)"));
  asserted &= problem->addPredicate(plansys2::Predicate("(robot_at cleaner clean_restaurant)"));
  if (!asserted) {
    return fail(node, "the problem expert refused where the robots are");
  }
  // What is wanted, classically: a valve shut. That the cleaner and the
  // concierge must come to know it, and the guest must not learn the room,
  // is the whole difficulty and none of it fits in this line.
  std::string room = leak;
  for (auto & ch : room) {
    ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
  }
  problem->setGoal(plansys2::Goal("(and (shut_off porter " + room + "))"));

  RCLCPP_INFO(node->get_logger(), "[mission] planning for the %s fleet", fleet.c_str());
  const auto started = node->now();
  auto plan = planner->getPlan(domain->getDomain(), problem->getProblem());
  const double planning = (node->now() - started).seconds();
  if (!plan.has_value() || plan->items.empty()) {
    return fail(node, "no policy after " + std::to_string(planning) + " s");
  }
  std::size_t leaves = 0;
  for (const auto & item : plan->items) {
    for (const auto child : item.children) {
      leaves += child == PlanItem::POLICY_DONE ? 1 : 0;
    }
  }
  RCLCPP_INFO(
    node->get_logger(), "[mission] policy with %zu nodes, %zu leaves, in %.1f s",
    plan->items.size(), leaves, planning);
  show(node->get_logger(), plan.value(), 0, "  ", "");
  if (!policy_out.empty()) {
    write_policy(policy_out, plan.value());
  }

  if (!executor->start_plan_execution(plan.value())) {
    return fail(node, "the executor refused the policy");
  }
  RCLCPP_INFO(node->get_logger(), "[mission] executing");
  rclcpp::Rate rate(4);
  while (rclcpp::ok() && executor->execute_and_check_plan()) {
    rclcpp::spin_some(node);
    rate.sleep();
  }
  const auto result = executor->getResult();
  if (!result || result->result != plansys2_msgs::action::ExecutePlan::Result::SUCCESS) {
    return fail(node, "the executor did not complete the policy");
  }
  RCLCPP_INFO(node->get_logger(), "[mission] the policy is done");

  // The verdict, at the actual world.
  rclcpp::sleep_for(2s);
  const auto view_text = await_message(node, "/hotel/knowledge", {"{"}, 30s);
  if (view_text.empty()) {
    return fail(node, "the knowledge view published nothing");
  }
  const auto view = nlohmann::json::parse(view_text);
  const Verdict got{
    view.at("safe").get<bool>(), view.at("stand_down").get<bool>(),
    view.at("secret").get<bool>()};

  // The epistemic state's own answer on the ontic conjunct, at its designated
  // worlds, which by now are copies of the actual one. The two must agree.
  const auto contained = holds(node, "safe");
  if (!contained.has_value()) {
    return fail(node, "the epistemic state could not say whether the leak is contained");
  }
  if (*contained != got.safe) {
    return fail(node, "the knowledge view and the epistemic state disagree on safe");
  }

  RCLCPP_INFO(
    node->get_logger(),
    "[mission] verdict: the leak contained: %s; every responder knows every responder "
    "knows it: %s; the guest cannot rule out any room: %s",
    yes(got.safe), yes(got.stand_down), yes(got.secret));

  ask("stand-down");
  const auto stood = await_message(
    node, "/hotel/crew", {"stood down", "stand-down failed", "stand-down refused"}, 1200s);
  if (stood.rfind("stood down", 0) != 0) {
    return fail(node, stood.empty() ? "no stand-down" : stood);
  }

  if (got.safe != expected.safe || got.stand_down != expected.stand_down ||
    got.secret != expected.secret)
  {
    return fail(
      node, std::string("the ") + fleet + " fleet was expected to give " +
      yes(expected.safe) + "/" + yes(expected.stand_down) + "/" + yes(expected.secret) +
      " and gave " + yes(got.safe) + "/" + yes(got.stand_down) + "/" + yes(got.secret));
  }
  const bool whole = got.safe && got.stand_down && got.secret;
  RCLCPP_INFO(
    node->get_logger(), "[mission] mission complete: the %s fleet %s the whole goal, as the "
    "planner and the trace say it does", fleet.c_str(), whole ? "meets" : "does not meet");
  rclcpp::shutdown();
  return 0;
}
