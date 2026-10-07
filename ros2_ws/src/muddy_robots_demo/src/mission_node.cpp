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

// Starts the muddy robots, on the floor with a public address or the one
// without.
//
// The robots first drive from their stations to the muster point; the model
// starts there, where each sees every other robot's lamp. The mission then
// asks the planner for a policy whose goal is that every robot knows whether
// it is faulty.
//
// On the floor with a public address there is one: the announcement, then
// bells, branching on whether anyone left. It is run, and then every robot is
// dismissed to do what it knows: the faulty to the calibration bay, the rest
// back to their stations.
//
// On the floor without one there is none, and the planner says so. The
// mission then rings the bell as many times as there are robots, written out
// here as a policy because no planner produced it, and dismisses them. Nobody
// knows anything at any bell, nobody moves, and at the dismissal every robot
// stays where it is; the mission confirms that no robot knows whether it is
// faulty before it reports that as the expected outcome.

#include <chrono>
#include <fstream>
#include <memory>
#include <optional>
#include <regex>
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
using plansys2_msgs::msg::PlanItem;

namespace
{

std::string read_file(const std::string & path)
{
  std::ifstream in(path);
  if (!in) {
    throw std::runtime_error("cannot read " + path);
  }
  return {std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
}

std::vector<std::string> agents_of(const std::string & text)
{
  const std::regex pattern{R"(\(\s*:agents\s+([^)]*)\))"};
  std::smatch found;
  std::vector<std::string> words;
  if (std::regex_search(text, found, pattern)) {
    std::istringstream in(found[1].str());
    for (std::string w; in >> w; ) {
      words.push_back(w);
    }
  }
  return words;
}

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

void show(
  const rclcpp::Logger & log, const plansys2_msgs::msg::Plan & plan, std::size_t index,
  const std::string & prefix, const std::string & label)
{
  if (index >= plan.items.size()) {
    return;
  }
  const auto & item = plan.items[index];
  RCLCPP_INFO(
    log, "[policy] %s%s%s%s", prefix.c_str(), label.c_str(), item.epistemic_action.c_str(),
    item.sensing ? "   [sensing]" : "");
  for (std::size_t i = 0; i < item.children.size(); ++i) {
    const auto child = item.children[i];
    const std::string outcome = i < item.outcomes.size() ? item.outcomes[i] : std::string{"?"};
    if (child == PlanItem::POLICY_DONE) {
      RCLCPP_INFO(log, "[policy] %s  %s -> done", prefix.c_str(), outcome.c_str());
      continue;
    }
    show(log, plan, child, prefix + "  ", item.children.size() > 1 ? outcome + " -> " : std::string{});
  }
}

void write_policy(const std::string & path, const plansys2_msgs::msg::Plan & plan, bool planned)
{
  nlohmann::json out;
  out["goal"] = plan.epistemic_goal;
  out["planned"] = planned;
  out["items"] = nlohmann::json::array();
  for (std::size_t i = 0; i < plan.items.size(); ++i) {
    const auto & item = plan.items[i];
    nlohmann::json children = nlohmann::json::array();
    for (const auto c : item.children) {
      children.push_back(c == PlanItem::POLICY_DONE ? -1 : static_cast<int>(c));
    }
    out["items"].push_back({{"index", i}, {"action", item.action},
        {"epistemic_action", item.epistemic_action}, {"sensing", item.sensing},
        {"children", children}, {"outcomes", item.outcomes},
        {"knowledge_requirements", item.knowledge_requirements}});
  }
  std::ofstream(path) << out.dump(2) << "\n";
}

/// The goal, as the executor reads formulas: every robot knows whether it is
/// faulty.
std::string goal_of(const std::vector<std::string> & agents)
{
  std::string out = "(and";
  for (const auto & a : agents) {
    out += " (Kw " + a + " faulty_" + a + ")";
  }
  return out + ")";
}

/// The silent floor's bells as a policy: the bell, rung `bells` times while
/// nobody leaves. No planner produced it; the planner says nothing would.
plansys2_msgs::msg::Plan bells_only(
  const nlohmann::json & mapping, int bells, const std::vector<std::string> & agents)
{
  plansys2_msgs::msg::Plan plan;
  plan.epistemic_goal = goal_of(agents);
  float clock = 0.0f;
  std::int64_t previous = -1;
  for (int k = 0; k < bells; ++k) {
    PlanItem item;
    item.epistemic_action = "bell_muster";
    item.action = mapping.at("bell_muster").at("action").get<std::string>();
    item.duration = mapping.at("bell_muster").at("duration").get<float>();
    item.time = clock;
    clock += item.duration + 0.001f;
    item.sensing = true;
    plan.items.push_back(item);
    const auto index = static_cast<std::uint32_t>(plan.items.size() - 1);
    if (previous >= 0) {
      plan.items[previous].children.push_back(index);
      plan.items[previous].outcomes.push_back("e-stay");
      plan.items[previous].children.push_back(PlanItem::POLICY_DONE);
      plan.items[previous].outcomes.push_back("e-leave");
    }
    previous = index;
  }
  plan.items[previous].children.push_back(PlanItem::POLICY_DONE);
  plan.items[previous].outcomes.push_back("e-stay");
  plan.items[previous].children.push_back(PlanItem::POLICY_DONE);
  plan.items[previous].outcomes.push_back("e-leave");
  return plan;
}

/// Asks the epistemic state whether a formula holds now. Empty when no answer
/// could be had, which is not the same as no.
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
  if (rclcpp::spin_until_future_complete(node, future, 5s) != rclcpp::FutureReturnCode::SUCCESS) {
    return std::nullopt;
  }
  const auto answer = future.get();
  if (!answer->success) {
    return std::nullopt;
  }
  return answer->holds;
}

/// Waits for the crew to say `what` on /muddy_robots/crew; returns what it said.
std::string await_crew(
  const rclcpp::Node::SharedPtr & node, const std::string & prefix, std::chrono::seconds patience)
{
  std::string heard;
  auto sub = node->create_subscription<std_msgs::msg::String>(
    "/muddy_robots/crew", rclcpp::QoS(10).transient_local().reliable(),
    [&](std_msgs::msg::String::SharedPtr m) {
      if (m->data.rfind(prefix, 0) == 0) {
        heard = m->data;
      }
    });
  const auto deadline = std::chrono::steady_clock::now() + patience;
  while (rclcpp::ok() && heard.empty() && std::chrono::steady_clock::now() < deadline) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(100ms);
  }
  return heard;
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("muddy_robots_mission");
  const auto problem_path = node->declare_parameter<std::string>("epddl_problem", "");
  const auto mapping_path = node->declare_parameter<std::string>("action_mapping", "");
  const auto floor = node->declare_parameter<std::string>("floor", "pa");
  const auto policy_out = node->declare_parameter<std::string>("policy_out", "");
  const double hold = node->declare_parameter<double>("hold", 0.0);

  std::vector<std::string> agents;
  nlohmann::json mapping;
  try {
    agents = agents_of(read_file(problem_path));
    mapping = nlohmann::json::parse(read_file(mapping_path));
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node->get_logger(), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  if (agents.empty()) {
    RCLCPP_ERROR(node->get_logger(), "%s names no agents", problem_path.c_str());
    rclcpp::shutdown();
    return 1;
  }

  auto request_pub = node->create_publisher<std_msgs::msg::String>(
    "/muddy_robots/request", rclcpp::QoS(10).reliable());
  // Asked once the crew has matched the topic, which is waited for below.
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
  for (const auto & name : {"domain_expert", "problem_expert", "epistemic_state", "planner", "executor"}) {
    if (!active(node, name, 240s)) {
      RCLCPP_ERROR(node->get_logger(), "%s never became active", name);
      rclcpp::shutdown();
      return 1;
    }
  }

  if (hold > 0.0) {
    RCLCPP_INFO(node->get_logger(), "[mission] holding %.0f s before the shift ends", hold);
    rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(hold * 1000)));
  }

  // The robots drive to the muster point; the model starts there.
  while (rclcpp::ok() && request_pub->get_subscription_count() == 0) {
    rclcpp::sleep_for(200ms);
  }
  ask("assemble");
  if (await_crew(node, "assembled", 400s).empty()) {
    RCLCPP_ERROR(node->get_logger(), "[mission] mission failed: the robots never assembled");
    rclcpp::shutdown();
    return 1;
  }

  problem->addInstance(plansys2::Instance{"muster", "hall"});
  problem->addPredicate(plansys2::Predicate("(open muster)"));
  for (const auto & a : agents) {
    problem->addInstance(plansys2::Instance{a, "robot"});
  }
  problem->setGoal(plansys2::Goal("(and (dismissed muster))"));

  RCLCPP_INFO(
    node->get_logger(), "[mission] planning on the %s floor: %zu robots, from %s",
    floor.c_str(), agents.size(), problem_path.c_str());
  const auto started = node->now();
  auto plan = planner->getPlan(domain->getDomain(), problem->getProblem());
  const double planning = (node->now() - started).seconds();

  const bool planned = plan.has_value() && !plan->items.empty();
  if (planned) {
    std::size_t leaves = 0;
    for (const auto & item : plan->items) {
      for (const auto child : item.children) {
        leaves += child == PlanItem::POLICY_DONE ? 1 : 0;
      }
    }
    RCLCPP_INFO(
      node->get_logger(), "[mission] policy with %zu nodes, %zu leaves, in %.1f s",
      plan->items.size(), leaves, planning);
  } else {
    RCLCPP_INFO(
      node->get_logger(), "[mission] no policy: the planner returned none after %.1f s",
      planning);
    if (floor != "silent") {
      RCLCPP_ERROR(node->get_logger(), "[mission] mission failed: no policy on the %s floor",
        floor.c_str());
      rclcpp::shutdown();
      return 1;
    }
    plan = bells_only(mapping, static_cast<int>(agents.size()), agents);
    RCLCPP_INFO(
      node->get_logger(), "[mission] ringing the bell instead, %zu times, as many as there are "
      "robots", agents.size());
  }
  show(node->get_logger(), plan.value(), 0, "  ", "");
  if (!policy_out.empty()) {
    write_policy(policy_out, plan.value(), planned);
  }

  if (!executor->start_plan_execution(plan.value())) {
    RCLCPP_ERROR(node->get_logger(), "the executor refused the policy");
    rclcpp::shutdown();
    return 1;
  }
  RCLCPP_INFO(node->get_logger(), "[mission] executing");
  rclcpp::Rate rate(4);
  while (rclcpp::ok() && executor->execute_and_check_plan()) {
    rclcpp::spin_some(node);
    rate.sleep();
  }
  const auto result = executor->getResult();
  const bool succeeded =
    result && result->result == plansys2_msgs::action::ExecutePlan::Result::SUCCESS;
  // On the silent floor the bells run out with the goal unmet, and the
  // executor says so; that is the outcome the planner predicted, and the
  // checks below decide whether it is that outcome and not another.
  if (!succeeded && planned) {
    RCLCPP_ERROR(node->get_logger(), "[mission] mission failed: the executor did not complete the "
      "policy");
    rclcpp::shutdown();
    return 1;
  }

  // Whether each robot knows whether it is faulty, as the epistemic state
  // answers: at every designated world, so whatever the lamps are. Which way
  // each knows is the crew's to say, since it alone has the floor.
  std::string knows;
  int kw = 0;
  for (const auto & a : agents) {
    const std::string atom = "faulty_" + a;
    const auto whether = holds(node, "(Kw " + a + " " + atom + ")");
    if (!whether) {
      RCLCPP_ERROR(node->get_logger(), "[mission] mission failed: the epistemic state could not "
        "say whether %s knows whether it is faulty", a.c_str());
      rclcpp::shutdown();
      return 1;
    }
    kw += *whether ? 1 : 0;
    knows += (knows.empty() ? "" : ", ") + a + (*whether ? " knows whether" : " does not");
  }
  ask("dismiss");
  const auto dismissed = await_crew(node, "dismissed", 400s);
  if (dismissed.empty() || dismissed.find("refused") != std::string::npos ||
    dismissed.find("the model is wrong") != std::string::npos)
  {
    RCLCPP_ERROR(node->get_logger(), "[mission] mission failed: the crew refused the dismissal: "
      "%s", dismissed.empty() ? "no answer" : dismissed.c_str());
    rclcpp::shutdown();
    return 1;
  }

  if (planned && kw == static_cast<int>(agents.size())) {
    RCLCPP_INFO(
      node->get_logger(), "[mission] mission complete: every robot knows whether it is faulty, "
      "as the policy's goal says");
  } else if (!planned && kw == 0) {
    RCLCPP_INFO(
      node->get_logger(), "[mission] mission complete: after %zu bells no robot knows whether it "
      "is faulty, as the planner said; nobody moved", agents.size());
  } else {
    RCLCPP_ERROR(node->get_logger(), "[mission] mission failed: %s", knows.c_str());
    rclcpp::shutdown();
    return 1;
  }
  RCLCPP_INFO(node->get_logger(), "[mission] %s", dismissed.c_str());
  rclcpp::shutdown();
  return 0;
}
