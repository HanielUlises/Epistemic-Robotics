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

// Starts the multi-site survey, and says what the planner gave it.
//
// The same three steps anyone would type into `ros2 plansys2 terminal`: say
// what exists, say what is wanted, run. The difference from the single-site
// mission is that what exists now includes places -- three sites, which the
// classical half knows only as objects to pass around, and the epistemic half
// knows as the three worlds it cannot tell apart.
//
// `plan_only:=true` stops after printing the policy. The policy is the
// measurement this domain is interesting for -- eight nodes, six levels deep,
// found after 838 168 expansions -- and it is worth being able to take that
// measurement without forty minutes of simulator.

#include <fstream>
#include <memory>
#include <regex>
#include <sstream>
#include <string>
#include <vector>

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "plansys2_msgs/action/execute_plan.hpp"
#include "plansys2_msgs/msg/plan.hpp"
#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "plansys2_domain_expert/DomainExpertClient.hpp"

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;   // NOLINT (build/namespaces)

namespace
{

// The agents of the domain. `observer` is here because the goal excludes it:
// a conjunct saying an agent must not come to know is only meaningful if that
// agent exists and could have been told.
const std::vector<std::string> AGENTS = {"scout", "relay", "observer"};

/// The sites, read from the EPDDL problem the planner and the epistemic state
/// are given.
///
/// Not a list held here. Three files already name these sites -- the EPDDL
/// problem, the bridge's task map, and the launch that places their pallets --
/// and a fourth copy in this one was enough to lose a run. The sites were
/// renamed everywhere except here, so this node declared the old names as
/// objects while the executor was handed a policy over the new ones, and the
/// over-all condition of the first scan named a site that did not exist. The
/// executor then prints
///
///     Error checking over all reqs: (and (on_site scout a17))
///
/// fifty times a second for as long as you leave it, and nothing anywhere says
/// that `a17` is not an object of the problem.
std::vector<std::string> sites_of(const std::string & epddl_problem)
{
  std::ifstream source(epddl_problem);
  if (!source) {
    throw std::runtime_error("cannot read " + epddl_problem);
  }
  const std::string text{
    std::istreambuf_iterator<char>(source), std::istreambuf_iterator<char>()};

  // `(:objects a17 a31 a06 - site)`.
  const std::regex objects{R"(\(\s*:objects\s+([^)]*)\))"};
  std::smatch found;
  if (!std::regex_search(text, found, objects)) {
    throw std::runtime_error(epddl_problem + " declares no (:objects ...)");
  }

  std::vector<std::string> sites, pending;
  std::istringstream words(found[1].str());
  for (std::string word; words >> word; ) {
    if (word == "-") {
      std::string type;
      words >> type;
      if (type == "site") {
        sites.insert(sites.end(), pending.begin(), pending.end());
      }
      pending.clear();
      continue;
    }
    pending.push_back(word);
  }

  if (sites.empty()) {
    throw std::runtime_error(
      epddl_problem + " declares no objects of type site, so there is nothing "
      "for the mission to survey.");
  }
  return sites;
}

/// Wait until a lifecycle node of the bringup is actually active.
///
/// Not a refinement of "wait for the domain expert to answer", which is what
/// the single-site mission does and what this did first. A configured node
/// answers: the domain expert returns the domain while the bringup is still in
/// its configure pass, several seconds before anything is activated. Asking
/// for a plan at that moment is what broke this mission.
///
/// The failure is worth spelling out because it is the domain that causes it.
/// The whole planning system is one process, and a plan request is served on
/// the same executor the bringup's own activation calls arrive on. Solving the
/// single-site problem is forty expansions and returns before anyone notices.
/// Solving this one is 838 168 expansions and holds the process for seven
/// seconds, during which the lifecycle manager's activate calls go unanswered
/// and the bringup gives up with "Failed to start plansys2!". The planner then
/// returns a perfectly good policy to a system that has already shut down, and
/// the mission fails at `start_plan_execution` with "send_goal failed" --
/// forty lines and several seconds away from the thing that actually went
/// wrong.
bool active(
  const rclcpp::Node::SharedPtr & node, const std::string & lifecycle_node,
  std::chrono::seconds patience)
{
  auto client = node->create_client<lifecycle_msgs::srv::GetState>(
    lifecycle_node + "/get_state");

  const auto deadline = std::chrono::steady_clock::now() + patience;
  while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
    if (client->wait_for_service(500ms)) {
      auto future = client->async_send_request(
        std::make_shared<lifecycle_msgs::srv::GetState::Request>());
      if (rclcpp::spin_until_future_complete(node, future, 2s) ==
        rclcpp::FutureReturnCode::SUCCESS)
      {
        if (future.get()->current_state.id ==
          lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
        {
          return true;
        }
      }
    }
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(500ms);
  }
  return false;
}

/// Print the policy as a tree, since that is what it is.
///
/// A list of eight numbered lines is what the message holds and not what the
/// planner found. What it found is a branch: after each scan the team does one
/// thing or another depending on what the laser returned, and the two leaves
/// of the second branch are the interesting ones -- in one of them the team
/// ends up knowing about a site no robot has been to.
void show(
  const rclcpp::Logger & log, const plansys2_msgs::msg::Plan & plan,
  std::size_t index, const std::string & prefix, const std::string & label)
{
  if (index >= plan.items.size()) {
    return;
  }
  const auto & item = plan.items[index];
  RCLCPP_INFO(
    log, "%s%s%s%s", prefix.c_str(), label.c_str(), item.action.c_str(),
    item.sensing ? "   [sensing]" : "");

  for (std::size_t i = 0; i < item.children.size(); ++i) {
    const auto child = item.children[i];
    const std::string outcome =
      i < item.outcomes.size() ? item.outcomes[i] : std::string{"?"};
    if (child == plansys2_msgs::msg::PlanItem::POLICY_DONE) {
      RCLCPP_INFO(log, "%s  %s -> done", prefix.c_str(), outcome.c_str());
      continue;
    }
    show(
      log, plan, child, prefix + "  ",
      item.children.size() > 1 ? outcome + " -> " : std::string{});
  }
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("survey_sites_mission");
  node->declare_parameter("plan_only", false);
  // Where to write the policy, so that a figure of it is the policy and not a
  // drawing of one. Empty writes nothing.
  node->declare_parameter("policy_out", std::string{});
  // The EPDDL problem, so the sites this node declares as objects are the
  // sites the planner solved over. See `sites_of`.
  node->declare_parameter("epddl_problem", std::string{});
  const bool plan_only = node->get_parameter("plan_only").as_bool();
  const auto policy_out = node->get_parameter("policy_out").as_string();
  const auto epddl_problem = node->get_parameter("epddl_problem").as_string();

  if (epddl_problem.empty()) {
    RCLCPP_ERROR(
      node->get_logger(),
      "epddl_problem is not set. The sites are read from it rather than held "
      "here, so without it this node has nothing to declare.");
    rclcpp::shutdown();
    return 1;
  }

  std::vector<std::string> sites;
  try {
    sites = sites_of(epddl_problem);
  } catch (const std::exception & problem) {
    RCLCPP_ERROR(node->get_logger(), "%s", problem.what());
    rclcpp::shutdown();
    return 1;
  }

  auto problem = std::make_shared<plansys2::ProblemExpertClient>();
  auto planner = std::make_shared<plansys2::PlannerClient>();
  auto domain = std::make_shared<plansys2::DomainExpertClient>();
  auto executor = std::make_shared<plansys2::ExecutorClient>();

  // The bringup is lifecycle-managed and comes up on its own schedule. Every
  // node it manages has to be active before anything is asked of it -- see the
  // note on `active` above for what happens when the planner is asked early.
  RCLCPP_INFO(node->get_logger(), "waiting for the planning system");
  for (const auto & lifecycle_node :
    {"domain_expert", "problem_expert", "epistemic_state", "planner", "executor"})
  {
    if (!active(node, lifecycle_node, 240s)) {
      RCLCPP_ERROR(
        node->get_logger(), "%s never became active", lifecycle_node);
      rclcpp::shutdown();
      return 1;
    }
  }
  RCLCPP_INFO(node->get_logger(), "the planning system is active");

  // What exists. Nothing here says which site is contaminated, and nothing
  // here could: that is not a fact the problem expert holds, and the whole
  // reason the mission needs an epistemic layer is that at this point nobody
  // knows it -- neither the robots nor the process starting them.
  for (const auto & agent : AGENTS) {
    problem->addInstance(plansys2::Instance{agent, "robot"});
    problem->addPredicate(plansys2::Predicate("(at_depot " + agent + ")"));
  }
  for (const auto & site : sites) {
    problem->addInstance(plansys2::Instance{site, "site"});
  }

  // What is wanted, classically. The classical goal is nearly empty of
  // content, and deliberately: which channel carried the finding, and who was
  // allowed to hear it, is not expressible in PDDL at all. It travels with the
  // EPDDL the planner grounds and solves, and the policy that comes back is
  // answering that goal rather than this one.
  // Any site will do: the classical goal is nearly empty of content and is
  // not the goal the policy answers.
  problem->setGoal(plansys2::Goal("(and(told scout " + sites.front() + "))"));

  RCLCPP_INFO(
    node->get_logger(), "planning over %zu sites from %s",
    sites.size(), epddl_problem.c_str());
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
    node->get_logger(), "policy with %zu nodes, %zu leaves, %s",
    plan->items.size(), leaves, branches ? "branching" : "linear");
  show(node->get_logger(), plan.value(), 0, "  ", "");

  if (!policy_out.empty()) {
    std::ofstream out(policy_out);
    if (!out) {
      RCLCPP_ERROR(
        node->get_logger(), "could not write the policy to %s",
        policy_out.c_str());
    } else {
      out << "{\n  \"goal\": \"" << plan->epistemic_goal << "\",\n"
          << "  \"items\": [\n";
      for (std::size_t i = 0; i < plan->items.size(); ++i) {
        const auto & item = plan->items[i];
        out << "    {\"index\": " << i
            << ", \"action\": \"" << item.action
            << "\", \"epistemic_action\": \"" << item.epistemic_action
            << "\", \"sensing\": " << (item.sensing ? "true" : "false")
            << ", \"children\": [";
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
      RCLCPP_INFO(
        node->get_logger(), "policy written to %s", policy_out.c_str());
    }
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

  RCLCPP_INFO(node->get_logger(), "executing");
  rclcpp::Rate rate(4);
  while (rclcpp::ok() && executor->execute_and_check_plan()) {
    rclcpp::spin_some(node);
    rate.sleep();
  }

  const auto result = executor->getResult();
  const bool succeeded =
    result && result->result == plansys2_msgs::action::ExecutePlan::Result::SUCCESS;

  if (succeeded) {
    RCLCPP_INFO(node->get_logger(), "mission complete");
  } else {
    RCLCPP_ERROR(node->get_logger(), "mission failed");
  }

  rclcpp::shutdown();
  return succeeded ? 0 : 1;
}
