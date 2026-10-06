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


// Starts the false-belief demonstration, on one of its three floors.
//
// On every floor the mission asks the planner for a policy for `fetched`. The
// shift fixes the first two actions of any policy: the picker goes to its
// dock, and the mover carries the crate from t1 to t3.
//
// On the told floor the planner finds a policy in which the mover reports the
// move to the picker before the picker fetches. On the doubt floor it finds
// one in which the picker looks into t1 first. Either is run.
//
// On the untold floor there is none: the picker cannot come to believe where
// the crate is without a report, and looking where it believes the crate to
// be would leave it with no consistent belief, which the planner refuses. The
// mission then runs what the picker would do on its own beliefs, which is
// the false-belief task's question: after the shift's two actions, fetch the
// crate from t1. The executor checks the picker's belief that the crate is in
// t1, which holds, and dispatches; the performer finds t1 empty and refuses.
// The mission then asks the epistemic state whether the crate is in t3, the
// picker believes it in t1, and the mover believes the picker believes it in
// t1, and reports the floor as the expected outcome only if all three hold.

#include <fstream>
#include <memory>
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

std::vector<std::string> section(const std::string & text, const std::string & name)
{
  const std::regex pattern{R"(\(\s*:)" + name + R"(\s+([^)]*)\))"};
  std::smatch found;
  if (!std::regex_search(text, found, pattern)) {
    return {};
  }
  std::vector<std::string> words;
  std::istringstream in(found[1].str());
  for (std::string w; in >> w; ) {
    words.push_back(w);
  }
  return words;
}

std::vector<std::string> objects_of(const std::vector<std::string> & words, const std::string & type)
{
  std::vector<std::string> out, pending;
  for (std::size_t i = 0; i < words.size(); ++i) {
    if (words[i] == "-" && i + 1 < words.size()) {
      if (words[i + 1] == type) {
        out.insert(out.end(), pending.begin(), pending.end());
      }
      pending.clear();
      ++i;
      continue;
    }
    pending.push_back(words[i]);
  }
  return out;
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

/// What the picker would do on its own beliefs, after the shift's two actions:
/// fetch the crate from where it believes it is. Its knowledge requirement is
/// the modal conjunct of fetch's event, written as the planner writes it.
plansys2_msgs::msg::Plan own_beliefs(
  const nlohmann::json & mapping, const std::string & picker, const std::string & mover,
  const std::string & from, const std::string & to)
{
  plansys2_msgs::msg::Plan plan;
  plan.epistemic_goal = "fetched";
  float clock = 0.0f;
  const auto add = [&](const std::string & name, const std::vector<std::string> & reqs) {
      PlanItem item;
      item.epistemic_action = name;
      item.action = mapping.at(name).at("action").get<std::string>();
      item.duration = mapping.at(name).at("duration").get<float>();
      item.time = clock;
      clock += item.duration + 0.001f;
      item.sensing = false;
      item.knowledge_requirements = reqs;
      plan.items.push_back(item);
      return static_cast<std::uint32_t>(plan.items.size() - 1);
    };
  const auto link = [&](std::uint32_t a, std::uint32_t b, const std::string & event) {
      plan.items[a].children.push_back(b);
      plan.items[a].outcomes.push_back(event);
    };
  const auto dock = add("go-dock_" + picker, {});
  const auto move = add("relocate_" + mover + "_" + from + "_" + to, {});
  link(dock, move, "e-go-dock");
  const auto fetch = add("fetch_" + picker + "_" + from, {"(K " + picker + " crate-at_" + from + ")"});
  link(move, fetch, "e-relocate");
  link(fetch, PlanItem::POLICY_DONE, "e-fetch");
  return plan;
}

/// Ask the epistemic state whether a formula holds now. Unanswered is false.
bool holds(const rclcpp::Node::SharedPtr & node, const std::string & formula)
{
  auto client = node->create_client<plansys2_epistemic_msgs::srv::CheckFormula>(
    "epistemic_state/check_formula");
  if (!client->wait_for_service(5s)) {
    return false;
  }
  auto request = std::make_shared<plansys2_epistemic_msgs::srv::CheckFormula::Request>();
  request->formula = formula;
  auto future = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node, future, 5s) != rclcpp::FutureReturnCode::SUCCESS) {
    return false;
  }
  const auto answer = future.get();
  return answer->success && answer->holds;
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("false_belief_mission");
  const auto problem_path = node->declare_parameter<std::string>("epddl_problem", "");
  const auto mapping_path = node->declare_parameter<std::string>("action_mapping", "");
  const auto floor = node->declare_parameter<std::string>("floor", "told");
  const auto policy_out = node->declare_parameter<std::string>("policy_out", "");
  const bool plan_only = node->declare_parameter<bool>("plan_only", false);
  const double hold = node->declare_parameter<double>("hold", 0.0);
  const auto picker = node->declare_parameter<std::string>("picker", "picker");
  const auto mover = node->declare_parameter<std::string>("mover", "mover");
  const auto from = node->declare_parameter<std::string>("crate_from", "t1");
  const auto to = node->declare_parameter<std::string>("crate_to", "t3");

  std::vector<std::string> agents, bays;
  nlohmann::json mapping;
  try {
    const auto text = read_file(problem_path);
    agents = section(text, "agents");
    bays = objects_of(section(text, "objects"), "bay");
    mapping = nlohmann::json::parse(read_file(mapping_path));
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node->get_logger(), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }

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
  RCLCPP_INFO(node->get_logger(), "the planning system is active");

  if (hold > 0.0) {
    RCLCPP_INFO(node->get_logger(), "[mission] holding %.0f s before planning", hold);
    rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(hold * 1000)));
  }

  for (const auto & agent : agents) {
    problem->addInstance(plansys2::Instance{agent, "robot"});
    problem->addPredicate(plansys2::Predicate("(ready " + agent + ")"));
  }
  for (const auto & b : bays) {
    problem->addInstance(plansys2::Instance{b, "bay"});
  }
  problem->setGoal(plansys2::Goal("(and (fetched))"));

  RCLCPP_INFO(
    node->get_logger(), "[mission] planning on the %s floor: %zu agents, %zu bays, from %s",
    floor.c_str(), agents.size(), bays.size(), problem_path.c_str());
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
      node->get_logger(), "[mission] no policy for fetched: the planner returned none after "
      "%.1f s", planning);
    if (floor != "untold") {
      RCLCPP_ERROR(
        node->get_logger(), "[mission] mission failed: no policy on the %s floor", floor.c_str());
      rclcpp::shutdown();
      return 1;
    }
    plan = own_beliefs(mapping, picker, mover, from, to);
    RCLCPP_INFO(
      node->get_logger(), "[mission] running what the %s would do on its own beliefs: after the "
      "shift's two actions, fetch the crate from %s", picker.c_str(), from.c_str());
  }
  show(node->get_logger(), plan.value(), 0, "  ", "");
  if (!policy_out.empty()) {
    write_policy(policy_out, plan.value(), planned);
  }

  if (plan_only) {
    RCLCPP_INFO(node->get_logger(), "plan_only: not executing");
    rclcpp::shutdown();
    return 0;
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
  if (succeeded) {
    RCLCPP_INFO(node->get_logger(), "[mission] mission complete: fetched");
    rclcpp::shutdown();
    return 0;
  }
  if (!planned) {
    // Expected: the crate is in the second bay, the picker believes it in the
    // first, and the mover believes the picker believes so. The fetch was
    // dispatched on that belief and refused by the floor.
    const std::string at_to = "crate-at_" + to, at_from = "crate-at_" + from;
    const bool there = holds(node, at_to);
    const bool belief = holds(node, "(K " + picker + " " + at_from + ")");
    const bool attributed = holds(node, "(K " + mover + " (K " + picker + " " + at_from + "))");
    const bool shared = holds(node, "(K " + picker + " (K " + mover + " " + at_from + "))");
    const std::string verdict = "the crate is in " + to + ": " + (there ? "yes" : "no") +
      "; the " + picker + " believes it in " + from + ": " + (belief ? "yes" : "no") +
      "; the " + mover + " believes the " + picker + " believes so: " +
      (attributed ? "yes" : "no") + "; the " + picker + " believes the " + mover +
      " believes it in " + from + ": " + (shared ? "yes" : "no");
    if (there && belief && attributed) {
      RCLCPP_INFO(
        node->get_logger(), "[mission] mission complete: the %s went to %s, where it believed the "
        "crate was and the %s believed it would look, and found it empty; %s", picker.c_str(),
        from.c_str(), mover.c_str(), verdict.c_str());
      rclcpp::shutdown();
      return 0;
    }
    RCLCPP_ERROR(node->get_logger(), "[mission] mission failed: %s", verdict.c_str());
    rclcpp::shutdown();
    return 1;
  }
  RCLCPP_ERROR(node->get_logger(), "[mission] mission failed");
  rclcpp::shutdown();
  return 1;
}
