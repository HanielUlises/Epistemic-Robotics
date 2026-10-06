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

// Starts the pass-through mission, and says what the planner gave it.
//
// Say what exists, ask for a policy, run it. What exists is read from the EPDDL
// problem and not held here: the agents and the bays are named there, and the
// planner and the epistemic state both ground that file. A fourth list of
// names in this node is how the multi-site mission once declared objects the
// policy did not mention, and sat checking a condition over a site that did
// not exist fifty times a second without saying so.

#include <fstream>
#include <memory>
#include <regex>
#include <sstream>
#include <string>
#include <vector>

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_msgs/action/execute_plan.hpp"
#include "plansys2_msgs/msg/plan.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;   // NOLINT(build/namespaces)

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

/// The words of the first `(:<section> ...)` of the problem.
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

/// The objects of one type, from `(:objects a b - t c - u)`.
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

/// Wait until a lifecycle node of the bringup is active, and not merely
/// answering: a configured node answers several seconds before anything is
/// activated, and a plan requested then is handed to a system that is about
/// to report "Failed to start plansys2!".
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
    if (child == plansys2_msgs::msg::PlanItem::POLICY_DONE) {
      RCLCPP_INFO(log, "[policy] %s  %s -> done", prefix.c_str(), outcome.c_str());
      continue;
    }
    show(log, plan, child, prefix + "  ", item.children.size() > 1 ? outcome + " -> " : std::string{});
  }
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("pass_through_mission");
  const auto problem_path = node->declare_parameter<std::string>("epddl_problem", "");
  const auto policy_out = node->declare_parameter<std::string>("policy_out", "");
  const bool plan_only = node->declare_parameter<bool>("plan_only", false);
  // Seconds to hold before asking for a policy, so a recording opens on the
  // robots standing still with nothing known.
  const double hold = node->declare_parameter<double>("hold", 0.0);

  std::vector<std::string> agents, bays;
  try {
    const auto text = read_file(problem_path);
    agents = section(text, "agents");
    bays = objects_of(section(text, "objects"), "tunnel");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node->get_logger(), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  if (agents.empty() || bays.empty()) {
    RCLCPP_ERROR(
      node->get_logger(), "%s names %zu agents and %zu bays; the mission needs both",
      problem_path.c_str(), agents.size(), bays.size());
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

  // What exists. Nothing here says which bay is open: that is not a fact the
  // problem expert holds, and at this point nobody knows it.
  for (const auto & agent : agents) {
    problem->addInstance(plansys2::Instance{agent, "robot"});
    problem->addPredicate(plansys2::Predicate("(ready " + agent + ")"));
  }
  for (const auto & bay : bays) {
    problem->addInstance(plansys2::Instance{bay, "tunnel"});
  }
  // The classical goal. The goal the policy answers is the EPDDL one, which
  // the planner grounds itself.
  problem->setGoal(plansys2::Goal("(and (delivered))"));

  RCLCPP_INFO(
    node->get_logger(), "[mission] planning: %zu agents, %zu bays, from %s", agents.size(),
    bays.size(), problem_path.c_str());
  const auto started = node->now();
  const auto plan = planner->getPlan(domain->getDomain(), problem->getProblem());
  if (!plan.has_value()) {
    RCLCPP_ERROR(node->get_logger(), "no plan; is the epistemic solver configured?");
    rclcpp::shutdown();
    return 1;
  }

  std::size_t leaves = 0;
  bool branches = false;
  for (const auto & item : plan->items) {
    branches = branches || item.children.size() > 1;
    for (const auto child : item.children) {
      leaves += child == plansys2_msgs::msg::PlanItem::POLICY_DONE ? 1 : 0;
    }
  }
  RCLCPP_INFO(
    node->get_logger(), "[mission] policy with %zu nodes, %zu leaves, %s, in %.1f s",
    plan->items.size(), leaves, branches ? "branching" : "linear",
    (node->now() - started).seconds());
  show(node->get_logger(), plan.value(), 0, "  ", "");

  if (!policy_out.empty()) {
    std::ofstream out(policy_out);
    out << "{\n  \"goal\": \"" << plan->epistemic_goal << "\",\n  \"items\": [\n";
    for (std::size_t i = 0; i < plan->items.size(); ++i) {
      const auto & item = plan->items[i];
      out << "    {\"index\": " << i << ", \"action\": \"" << item.action
          << "\", \"epistemic_action\": \"" << item.epistemic_action
          << "\", \"sensing\": " << (item.sensing ? "true" : "false") << ", \"children\": [";
      for (std::size_t c = 0; c < item.children.size(); ++c) {
        out << (c ? ", " : "") << item.children[c];
      }
      out << "], \"outcomes\": [";
      for (std::size_t c = 0; c < item.outcomes.size(); ++c) {
        out << (c ? ", " : "") << '"' << item.outcomes[c] << '"';
      }
      out << "]}" << (i + 1 < plan->items.size() ? "," : "") << "\n";
    }
    out << "  ]\n}\n";
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
    RCLCPP_INFO(node->get_logger(), "[mission] mission complete");
  } else {
    RCLCPP_ERROR(node->get_logger(), "[mission] mission failed");
  }
  rclcpp::shutdown();
  return succeeded ? 0 : 1;
}
