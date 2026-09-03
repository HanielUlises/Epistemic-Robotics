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

// Starts the warehouse mission, so that the demo is one command.
//
// The same three steps anyone would type into `ros2 plansys2 terminal`: say
// what exists, say what is wanted, run.
//
// What is said below is the mechanical half. That the pallet is in one of two
// aisles and nobody knows which, that r1 may only lift it where it knows it
// is, and that r2 has to come to know without leaving receiving, is none of
// it expressible here. It travels with the EPDDL the epistemic solver grounds.

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
  auto node = rclcpp::Node::make_shared("warehouse_mission");

  auto problem = std::make_shared<plansys2::ProblemExpertClient>();
  auto planner = std::make_shared<plansys2::PlannerClient>();
  auto domain = std::make_shared<plansys2::DomainExpertClient>();
  auto executor = std::make_shared<plansys2::ExecutorClient>();

  RCLCPP_INFO(node->get_logger(), "waiting for the planning system");
  for (int i = 0; i < 120 && rclcpp::ok(); ++i) {
    if (!domain->getDomain().empty()) {
      break;
    }
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(500ms);
  }

  // The domain answering does not mean the problem expert accepts writes: they
  // are separate lifecycle nodes and the problem expert is usually the later.
  // Losing that race is quiet, because the epistemic solver reads the EPDDL
  // and plans anyway, and the executor is then handed a policy it cannot
  // dispatch.
  bool ready = false;
  for (int i = 0; i < 120 && rclcpp::ok(); ++i) {
    if (problem->addInstance(plansys2::Instance{"dock_south", "zone"})) {
      ready = true;
      break;
    }
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(500ms);
  }
  if (!ready) {
    RCLCPP_ERROR(node->get_logger(), "the problem expert never accepted a write");
    rclcpp::shutdown();
    return 1;
  }

  const auto assert_predicate = [&](const std::string & text) {
      if (!problem->addPredicate(plansys2::Predicate(text))) {
        RCLCPP_ERROR(node->get_logger(), "refused: %s", text.c_str());
        return false;
      }
      return true;
    };

  // The building. Which aisle holds the pallet is not a fact this expert
  // holds, and no predicate here could hold it.
  for (const auto & zone : {"corridor", "dock_north", "lane", "bay2", "bay3"}) {
    problem->addInstance(plansys2::Instance{zone, "zone"});
  }

  bool asserted = true;
  asserted &= assert_predicate("(is_bay bay2)");
  asserted &= assert_predicate("(is_bay bay3)");
  asserted &= assert_predicate("(is_dock dock_north)");

  // r1 comes on shift at shipping, r2 waits at receiving. r2 never moves, and
  // that is the instance rather than an oversight: it has to come to know what
  // r1 found without going to look.
  problem->addInstance(plansys2::Instance{"r1", "robot"});
  problem->addInstance(plansys2::Instance{"r2", "robot"});
  asserted &= assert_predicate("(robot_at r1 dock_south)");
  asserted &= assert_predicate("(robot_at r2 dock_north)");

  if (!asserted) {
    RCLCPP_ERROR(
      node->get_logger(),
      "the problem expert refused part of the warehouse, and a policy planned "
      "against the EPDDL would be dispatched into a world that does not match "
      "it. Stopping instead.");
    rclcpp::shutdown();
    return 1;
  }

  // What is wanted, classically: the pallet is at receiving. That r2 must know
  // which aisle it came out of is the other half, and it is in the EPDDL.
  problem->setGoal(plansys2::Goal("(and(delivered r1))"));

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

  // The executor is another lifecycle node on its own schedule, and asking it
  // before its action server is up fails the mission after the planning is
  // already done.
  bool started = false;
  for (int i = 0; i < 40 && rclcpp::ok(); ++i) {
    if (executor->start_plan_execution(plan.value())) {
      started = true;
      break;
    }
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(500ms);
  }
  if (!started) {
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
