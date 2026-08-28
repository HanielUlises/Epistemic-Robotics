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
//
// Starts the mission, so that the demonstration does not depend on someone
// typing into the PlanSys2 terminal.
//
// It does what an operator would do and nothing more: it tells the problem
// expert what exists and where the robot is, states the classical goal, asks
// for a plan and hands it to the executor. The plan that comes back is a
// policy, because the solver is the epistemic one and the problem it is
// really solving is the EPDDL pair named in the parameters -- this node never
// sees the branching, and could not have caused it.

#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "plansys2_msgs/action/execute_plan.hpp"
#include "plansys2_msgs/msg/plan.hpp"
#include "plansys2_pddl_parser/Utils.hpp"
#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"

using namespace std::chrono_literals;

class Mission : public rclcpp::Node
{
public:
  Mission()
  : Node("warehouse_mission")
  {
    // Waiting for a map before planning is not politeness. The policy's first
    // sensing action is checked against a model the perception node feeds from
    // the map, and starting before there is one only means the first drive
    // begins blind.
    declare_parameter<double>("settle", 12.0);

    // The fleet, and where each of its robots comes on shift.
    //
    // These have to agree with the EPDDL instance the planner is given: it
    // names the agents and their starting zones, and a policy that moves a
    // robot this node never declared is a policy the executor cannot
    // dispatch. The launch file reads both from the same place, so there is
    // one statement of the fleet and not two.
    declare_parameter<std::vector<std::string>>("agents", {"r1"});
    declare_parameter<std::vector<std::string>>("starts", {"dock_south"});

    // Who the epistemic half of the goal is about, for the caption alone. The
    // condition itself is `[Kw. ?i] (pallet-at bay2)` in the EPDDL problem and
    // is checked there; this node only has to be able to say whose knowledge
    // the run was for, and with three robots it is not the one that drove.
    declare_parameter<std::string>("knower", "r1");

    // The robot the classical goal is about: the one that fetches. See seed().
    declare_parameter<std::string>("deliverer", "r1");

    domain_ = std::make_shared<plansys2::DomainExpertClient>();
    problem_ = std::make_shared<plansys2::ProblemExpertClient>();
    planner_ = std::make_shared<plansys2::PlannerClient>();
    executor_ = std::make_shared<plansys2::ExecutorClient>();

    // What phase the mission is in, for the caption in rviz. Without this the
    // overlay has nothing to say until the executor dispatches its first
    // action -- a minute or more of Gazebo, SLAM and Nav2 coming up -- and a
    // caption that says nothing for a minute reads as something being wrong.
    phase_ = create_publisher<std_msgs::msg::String>(
      "/warehouse/phase", rclcpp::QoS(1).transient_local());
    say("STARTING UP: Gazebo, SLAM, Nav2, ePlanSys");

    timer_ = create_wall_timer(1s, [this] {step();});
  }

private:
  enum class Stage {Seeding, Planning, Running, Done};

  void say(const std::string & text)
  {
    std_msgs::msg::String message;
    message.data = text;
    phase_->publish(message);
  }

  void seed()
  {
    const auto agents = get_parameter("agents").as_string_array();
    const auto starts = get_parameter("starts").as_string_array();
    if (agents.size() != starts.size() || agents.empty()) {
      RCLCPP_ERROR(
        get_logger(),
        "agents (%zu) and starts (%zu) have to be the same non-empty length: "
        "each robot comes on shift somewhere", agents.size(), starts.size());
      throw std::runtime_error("agents and starts do not line up");
    }

    for (const auto & agent : agents) {
      problem_->addInstance(plansys2::Instance{agent, "robot"});
    }
    // The zones of the AWS warehouse floor, the same six the EPDDL instance
    // names: shipping and receiving at the two ends of the west corridor, the
    // service lane along the rack fronts, and the two aisles off it.
    for (const auto & zone :
      {"dock_south", "corridor", "dock_north", "lane", "bay2", "bay3"})
    {
      problem_->addInstance(plansys2::Instance{zone, "zone"});
    }

    for (std::size_t i = 0; i < agents.size(); ++i) {
      problem_->addPredicate(
        plansys2::Predicate("(robot_at " + agents[i] + " " + starts[i] + ")"));
    }
    problem_->addPredicate(plansys2::Predicate("(is_bay bay2)"));
    problem_->addPredicate(plansys2::Predicate("(is_bay bay3)"));
    problem_->addPredicate(plansys2::Predicate("(is_dock dock_north)"));

    // The classical goal: the pallet reaches a dock. What the mission is
    // really for -- that a robot which never went to look comes to *know*
    // which bay it came out of -- is in the EPDDL problem, and the executor
    // checks it at the end of the policy through CheckEpistemicGoal. Neither
    // goal is redundant: this one is about the pallet, that one is about a
    // robot's knowledge of it.
    //
    // It names one robot rather than saying "somebody delivered it". The
    // disjunction reads better and does not work: the epistemic solver spent
    // its whole sixty-second budget on `(or (delivered r1) (delivered r2))`
    // and returned no solution, for a task the same planner solves standalone
    // in a fifth of a second. Naming the robot that starts at shipping is not
    // a weaker mission -- it is what every policy the planner returns does
    // anyway, r1 being the one with the pallet in front of it -- and the
    // epistemic conjunct, which is the half that cannot be had by luck, is
    // untouched and still about a robot that never leaves its dock.
    problem_->setGoal(
      plansys2::Goal("(and (delivered " +
        get_parameter("deliverer").as_string() + "))"));
  }

  void step()
  {
    switch (stage_) {
      case Stage::Seeding: {
          if ((now() - started_).seconds() < get_parameter("settle").as_double()) {
            return;
          }
          seed();
          RCLCPP_INFO(get_logger(), "problem seeded; asking for a plan");
          say("PLANNING: asking ePlanSys for a policy");
          stage_ = Stage::Planning;
          break;
        }

      case Stage::Planning: {
          const auto domain = domain_->getDomain();
          const auto problem = problem_->getProblem();
          const auto plan = planner_->getPlan(domain, problem);

          if (!plan.has_value()) {
            RCLCPP_ERROR(get_logger(), "no plan; asking again");
            return;
          }

          std::size_t branching = 0;
          for (const auto & item : plan.value().items) {
            if (item.children.size() > 1) {++branching;}
          }
          RCLCPP_INFO(get_logger(),
            "policy: %zu nodes, %zu of them branching",
            plan.value().items.size(), branching);
          for (const auto & item : plan.value().items) {
            RCLCPP_INFO(get_logger(), "  %s%s", item.action.c_str(),
              item.children.size() > 1 ? "   <- branches on what it observes" : "");
          }

          if (!executor_->start_plan_execution(plan.value())) {
            RCLCPP_ERROR(get_logger(), "the executor refused the policy");
            return;
          }
          say("POLICY: " + std::to_string(plan.value().items.size()) +
            " nodes, branching");
          stage_ = Stage::Running;
          break;
        }

      case Stage::Running: {
          if (executor_->execute_and_check_plan()) {
            const auto feedback = executor_->getFeedBack();
            for (const auto & action : feedback.action_execution_status) {
              if (action.status == plansys2_msgs::msg::ActionExecutionInfo::EXECUTING &&
                action.action != running_)
              {
                running_ = action.action;
                RCLCPP_INFO(get_logger(), "executing %s", running_.c_str());
              }
            }
            return;
          }

          const auto result = executor_->getResult();
          const bool reached = result.has_value() &&
            result.value().result == plansys2_msgs::action::ExecutePlan::Result::SUCCESS;
          if (reached) {
            RCLCPP_INFO(get_logger(), "mission complete: the policy reached its goal");
            say("DONE: delivered, and " +
              get_parameter("knower").as_string() + " knows which bay");
          } else {
            RCLCPP_ERROR(get_logger(), "the policy did not reach its goal");
            for (const auto & action : executor_->getFeedBack().action_execution_status) {
              if (action.status == plansys2_msgs::msg::ActionExecutionInfo::FAILED) {
                RCLCPP_ERROR(get_logger(), "  %s failed: %s",
                  action.action.c_str(), action.message_status.c_str());
              }
            }
          }
          stage_ = Stage::Done;
          break;
        }

      case Stage::Done:
        break;
    }
  }

  std::shared_ptr<plansys2::DomainExpertClient> domain_;
  std::shared_ptr<plansys2::ProblemExpertClient> problem_;
  std::shared_ptr<plansys2::PlannerClient> planner_;
  std::shared_ptr<plansys2::ExecutorClient> executor_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr phase_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time started_{now()};
  Stage stage_{Stage::Seeding};
  std::string running_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Mission>());
  rclcpp::shutdown();
  return 0;
}
