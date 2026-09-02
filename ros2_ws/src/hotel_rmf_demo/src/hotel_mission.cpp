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

// Starts the hotel incident, so that the demo is one command.
//
// The same three steps anyone would type into `ros2 plansys2 terminal`: say
// what exists, say what is wanted, run.
//
// What is said here is only the mechanical half. That a leak is in one of two
// suites and nobody knows which, that the guest must not learn which, that
// shutting a valve in a closed suite leaves the porter believing something
// false --- none of it appears below, because none of it can. It travels with
// the EPDDL the epistemic solver grounds and solves.

#include <memory>
#include <string>

#include "plansys2_msgs/action/execute_plan.hpp"
#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "plansys2_domain_expert/DomainExpertClient.hpp"

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;   // NOLINT (build/namespaces)

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("hotel_mission");

  auto problem = std::make_shared<plansys2::ProblemExpertClient>();
  auto planner = std::make_shared<plansys2::PlannerClient>();
  auto domain = std::make_shared<plansys2::DomainExpertClient>();
  auto executor = std::make_shared<plansys2::ExecutorClient>();

  // The bringup is lifecycle-managed and the experts activate on their own
  // schedule. A domain that answers does not mean a problem expert that
  // accepts writes: they are separate nodes and the problem expert is usually
  // the later of the two. Waiting only for the domain is a race, and losing it
  // is quiet -- the instances are refused one by one, the epistemic solver
  // plans anyway because it reads the EPDDL and not the problem, and the
  // executor then has a policy it cannot dispatch.
  RCLCPP_INFO(node->get_logger(), "waiting for the planning system");
  for (int i = 0; i < 120 && rclcpp::ok(); ++i) {
    if (!domain->getDomain().empty()) {
      break;
    }
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(500ms);
  }

  // So write one instance and keep trying until it takes.
  bool problem_ready = false;
  for (int i = 0; i < 120 && rclcpp::ok(); ++i) {
    if (problem->addInstance(plansys2::Instance{"lobby", "zone"})) {
      problem_ready = true;
      break;
    }
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(500ms);
  }
  if (!problem_ready) {
    RCLCPP_ERROR(node->get_logger(), "the problem expert never accepted a write");
    rclcpp::shutdown();
    return 1;
  }

  // The building. Two suites, one on each of the upper floors, and the lobby
  // the lifts open into. Which suite is flooded is not a fact this expert
  // holds, and there is no predicate here that could hold it.
  for (const auto & zone : {"l2_suite", "l3_suite"}) {
    problem->addInstance(plansys2::Instance{zone, "zone"});
  }
  const auto assert_predicate = [&](const std::string & text) {
      if (!problem->addPredicate(plansys2::Predicate(text))) {
        RCLCPP_ERROR(node->get_logger(), "refused: %s", text.c_str());
        return false;
      }
      return true;
    };

  bool asserted = true;
  asserted &= assert_predicate("(is_desk lobby)");
  asserted &= assert_predicate("(is_suite l2_suite)");
  asserted &= assert_predicate("(is_suite l3_suite)");

  // The fleet, on shift in the lobby. `guest` is an agent of the epistemic
  // model and not a robot: it is who the mission must not inform, and it never
  // moves. It is declared here so the executor can name it in a radio call it
  // will never make.
  for (const auto & robot : {"inspector", "porter", "guest"}) {
    problem->addInstance(plansys2::Instance{robot, "robot"});
    asserted &= assert_predicate(
      "(robot_at " + std::string(robot) + " lobby)");
  }

  if (!asserted) {
    RCLCPP_ERROR(
      node->get_logger(),
      "the problem expert refused part of the building, and a policy planned "
      "against the EPDDL would be dispatched into a world that does not match "
      "it. Stopping instead.");
    rclcpp::shutdown();
    return 1;
  }

  // What is wanted, classically: a valve has been shut somewhere. That the
  // porter must come to know it, and the guest must not learn which suite,
  // is the whole difficulty and none of it fits in this line.
  problem->setGoal(plansys2::Goal("(and(shut_off inspector l2_suite))"));

  RCLCPP_INFO(node->get_logger(), "planning");
  const auto plan = planner->getPlan(domain->getDomain(), problem->getProblem());
  if (!plan.has_value()) {
    RCLCPP_ERROR(node->get_logger(), "no plan; is the epistemic solver configured?");
    rclcpp::shutdown();
    return 1;
  }

  bool branches = false;
  for (const auto & item : plan->items) {
    branches = branches || item.children.size() > 1;
  }
  RCLCPP_INFO(
    node->get_logger(), "policy with %zu nodes, %s",
    plan->items.size(), branches ? "branching" : "linear");

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
