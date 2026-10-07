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

// Starts the coordinated attack, on one of its four floors.
//
// On every floor the mission asks the planner for a policy for `lifted`.
//
// On the beacon floor there is one, and it is run. On the sight floor, where
// the robots' positions are not announced, there is one too: it sights the
// other robot through t2 before it signals, and it is run.
//
// On the radio floor there is none: the planner exhausts its space and says so.
// The mission then runs the best the radio can do, written out here as a
// policy because no planner produced it: read the order, then the four
// messages, each one level deeper, then lift. Every message is executed and
// applied to the model like any planned action, with its knowledge
// precondition checked first. The lift is checked the same way, against
// C{south,north} job(s), and the executor refuses it. That refusal is the
// result of this floor, and the mission reports it as the expected outcome and
// not as a fault.
//
// The blind floor is the sight floor with crates across t2. There is no
// policy there either, and the mission runs the sight floor's policy less the
// sighting: both robots to their viewpoints, the order read, the signal, the
// lift. The listener sees the signal, but the announcer cannot tell that it
// did, so the signal gives E^1 and no more, and the executor refuses the lift.
//
// With more than two robots, from tools/scaled.py, the beacon floor is run as
// before, and the lift needs C over all of them. On the radio floor the
// protocol is not written here: it is the one tools/ladder.py had the planner
// find, the fewest messages that reach E^k for the robots on the floor, and it
// is given as `protocol`. Each message requires that its sender knows the
// content, K_i E^(l-1) job(s), and at the end the mission confirms that E^k
// holds and C does not before it reports the refusal of lift as expected.

#include <fstream>
#include <memory>
#include <regex>
#include <sstream>
#include <string>
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

/// The radio protocol as a policy: read the order, then on each outcome the
/// four messages for the stand it names, then lift. Its knowledge requirements
/// are the modal conjuncts of each action's designated event, written as the
/// planner writes them.
plansys2_msgs::msg::Plan radio_protocol(
  const nlohmann::json & mapping, const std::string & reader, const std::string & other,
  const std::vector<std::string> & stands)
{
  plansys2_msgs::msg::Plan plan;
  plan.epistemic_goal = "lifted";
  float clock = 0.0f;

  const auto add = [&](const std::string & name, const std::vector<std::string> & reqs,
      bool sensing) -> std::uint32_t {
      PlanItem item;
      item.epistemic_action = name;
      item.action = mapping.at(name).at("action").get<std::string>();
      item.duration = mapping.at(name).at("duration").get<float>();
      item.time = clock;
      clock += item.duration + 0.001f;
      item.sensing = sensing;
      item.knowledge_requirements = reqs;
      plan.items.push_back(item);
      return static_cast<std::uint32_t>(plan.items.size() - 1);
    };
  const auto link = [&](std::uint32_t from, std::uint32_t to, const std::string & event) {
      plan.items[from].children.push_back(to);
      plan.items[from].outcomes.push_back(event);
    };

  // The order is read about the first stand; the second outcome means the
  // other one, since the order names exactly one.
  const auto root = add("read-order_" + reader + "_" + stands.at(0), {}, true);
  for (std::size_t k = 0; k < 2; ++k) {
    const std::string s = stands.at(k);
    const std::string job = "job_" + s;
    const std::string a = reader, b = other;
    const std::string k1 = "(K " + a + " " + job + ")";
    const std::string k2 = "(K " + b + " " + k1 + ")";
    const std::string k3 = "(K " + a + " " + k2 + ")";
    const std::string k4 = "(K " + b + " " + k3 + ")";
    const auto tell = add("tell_" + a + "_" + b + "_" + s, {k1}, false);
    link(root, tell, k == 0 ? "e-here" : "e-elsewhere");
    const auto ack = add("ack_" + b + "_" + a + "_" + s, {k2}, false);
    link(tell, ack, "e-tell");
    const auto ack2 = add("ack2_" + a + "_" + b + "_" + s, {k3}, false);
    link(ack, ack2, "e-ack");
    const auto ack3 = add("ack3_" + b + "_" + a + "_" + s, {k4}, false);
    link(ack2, ack3, "e-ack2");
    const auto lift = add("lift_" + s, {"(C (" + a + " " + b + ") " + job + ")"}, false);
    link(ack3, lift, "e-ack3");
    link(lift, PlanItem::POLICY_DONE, "e-lift");
  }
  return plan;
}

/// The sight floor's policy without its sighting, which the blind floor
/// cannot perform: the other robot to its viewpoint, the order read, the
/// reader to its viewpoint, the signal, the lift.
plansys2_msgs::msg::Plan blind_protocol(
  const nlohmann::json & mapping, const std::string & reader, const std::string & other,
  const std::vector<std::string> & stands)
{
  plansys2_msgs::msg::Plan plan;
  plan.epistemic_goal = "lifted";
  float clock = 0.0f;

  const auto add = [&](const std::string & name, const std::vector<std::string> & reqs,
      bool sensing) -> std::uint32_t {
      PlanItem item;
      item.epistemic_action = name;
      item.action = mapping.at(name).at("action").get<std::string>();
      item.duration = mapping.at(name).at("duration").get<float>();
      item.time = clock;
      clock += item.duration + 0.001f;
      item.sensing = sensing;
      item.knowledge_requirements = reqs;
      plan.items.push_back(item);
      return static_cast<std::uint32_t>(plan.items.size() - 1);
    };
  const auto link = [&](std::uint32_t from, std::uint32_t to, const std::string & event) {
      plan.items[from].children.push_back(to);
      plan.items[from].outcomes.push_back(event);
    };

  const auto root = add("go-view_" + other, {}, false);
  const auto read = add("read-order_" + reader + "_" + stands.at(0), {}, true);
  link(root, read, "e-go-view");
  for (std::size_t k = 0; k < 2; ++k) {
    const std::string s = stands.at(k);
    const std::string job = "job_" + s;
    const auto go = add("go-view_" + reader, {}, false);
    link(read, go, k == 0 ? "e-here" : "e-elsewhere");
    const auto signal = add(
      "signal_" + reader + "_" + other + "_" + s, {"(K " + reader + " " + job + ")"}, true);
    link(go, signal, "e-go-view");
    const auto lift = add("lift_" + s, {"(C (" + reader + " " + other + ") " + job + ")"}, false);
    link(signal, lift, "e-signal-seen");
    link(lift, PlanItem::POLICY_DONE, "e-lift");
  }
  return plan;
}

/// E^k phi over the group, as the executor reads formulas: it has no E, and
/// E is every agent's K.
std::string everyone(int k, const std::string & phi, const std::vector<std::string> & group)
{
  if (k == 0) {
    return phi;
  }
  const auto inner = everyone(k - 1, phi, group);
  std::string out = "(and";
  for (const auto & a : group) {
    out += " (K " + a + " " + inner + ")";
  }
  return out + ")";
}

std::string common(const std::vector<std::string> & group, const std::string & phi)
{
  std::string names;
  for (const auto & a : group) {
    names += (names.empty() ? "" : " ") + a;
  }
  return "(C (" + names + ") " + phi + ")";
}

/// The level of a message from its kind: tell 1, ack 2, ack2 3, ...
int level_of(const std::string & kind)
{
  if (kind == "tell") {
    return 1;
  }
  if (kind == "ack") {
    return 2;
  }
  return std::stoi(kind.substr(3)) + 1;
}

/// The radio protocol tools/ladder.py found, as a policy: read the order,
/// then on each outcome that branch's messages, then lift. Each message
/// requires that its sender knows what it says, K_i E^(l-1) job(s).
plansys2_msgs::msg::Plan found_protocol(
  const nlohmann::json & mapping, const std::string & reader,
  const std::vector<std::string> & agents, const std::vector<std::string> & stands,
  const nlohmann::json & protocol)
{
  plansys2_msgs::msg::Plan plan;
  plan.epistemic_goal = "lifted";
  float clock = 0.0f;

  const auto add = [&](const std::string & name, const std::vector<std::string> & reqs,
      bool sensing) -> std::uint32_t {
      PlanItem item;
      item.epistemic_action = name;
      item.action = mapping.at(name).at("action").get<std::string>();
      item.duration = mapping.at(name).at("duration").get<float>();
      item.time = clock;
      clock += item.duration + 0.001f;
      item.sensing = sensing;
      item.knowledge_requirements = reqs;
      plan.items.push_back(item);
      return static_cast<std::uint32_t>(plan.items.size() - 1);
    };
  const auto link = [&](std::uint32_t from, std::uint32_t to, const std::string & event) {
      plan.items[from].children.push_back(to);
      plan.items[from].outcomes.push_back(event);
    };

  const auto root = add("read-order_" + reader + "_" + stands.at(0), {}, true);
  for (std::size_t k = 0; k < 2; ++k) {
    const std::string s = stands.at(k);
    const std::string job = "job_" + s;
    auto from = root;
    std::string event = k == 0 ? "e-here" : "e-elsewhere";
    for (const auto & entry : protocol.at(s)) {
      // kind_sender_receiver_stand; no agent or stand name has an underscore.
      const auto name = entry.get<std::string>();
      const auto first = name.find('_');
      const auto kind = name.substr(0, first);
      const auto sender = name.substr(first + 1, name.find('_', first + 1) - first - 1);
      const auto message = add(
        name, {"(K " + sender + " " + everyone(level_of(kind) - 1, job, agents) + ")"}, false);
      link(from, message, event);
      from = message;
      event = "e-" + kind;
    }
    const auto lift = add("lift_" + s, {common(agents, job)}, false);
    link(from, lift, event);
    link(lift, PlanItem::POLICY_DONE, "e-lift");
  }
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
  auto node = rclcpp::Node::make_shared("coordinated_attack_mission");
  const auto problem_path = node->declare_parameter<std::string>("epddl_problem", "");
  const auto mapping_path = node->declare_parameter<std::string>("action_mapping", "");
  const auto floor = node->declare_parameter<std::string>("floor", "beacon");
  const auto policy_out = node->declare_parameter<std::string>("policy_out", "");
  const bool plan_only = node->declare_parameter<bool>("plan_only", false);
  const auto protocol_path = node->declare_parameter<std::string>("protocol", "");
  const double hold = node->declare_parameter<double>("hold", 0.0);

  std::vector<std::string> agents, stands;
  nlohmann::json mapping, protocol;
  try {
    const auto text = read_file(problem_path);
    agents = section(text, "agents");
    stands = objects_of(section(text, "objects"), "stand");
    mapping = nlohmann::json::parse(read_file(mapping_path));
    if (!protocol_path.empty()) {
      protocol = nlohmann::json::parse(read_file(protocol_path));
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node->get_logger(), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  // The positions floors and the published radio protocol are written for
  // two robots; the beacon floor, and the radio floor with a protocol from
  // tools/ladder.py, for any number.
  const bool two_only = floor == "sight" || floor == "blind" ||
    (floor == "radio" && protocol.is_null());
  if (agents.size() < 2 || (two_only && agents.size() != 2) || stands.size() != 2) {
    RCLCPP_ERROR(
      node->get_logger(), "%s names %zu agents and %zu stands; the %s floor needs %s robots and "
      "two stands%s", problem_path.c_str(), agents.size(), stands.size(), floor.c_str(),
      two_only ? "two" : "at least two",
      floor == "radio" && protocol.is_null() ? ", or a protocol from tools/ladder.py" : "");
    rclcpp::shutdown();
    return 1;
  }

  // The reader is the agent the problem says reads the order.
  std::string reader = agents[0];
  {
    const auto text = read_file(problem_path);
    const std::regex reads{R"(\(\[C\. All\] \(reads-order (\w+)\)\))"};
    std::smatch who;
    if (std::regex_search(text, who, reads)) {
      reader = who[1].str();
    }
  }
  const std::string other = reader == agents[0] ? agents[1] : agents[0];

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
  for (const auto & s : stands) {
    problem->addInstance(plansys2::Instance{s, "stand"});
  }
  problem->setGoal(plansys2::Goal("(and (lifted))"));

  RCLCPP_INFO(
    node->get_logger(), "[mission] planning on the %s floor: %zu agents, %zu stands, from %s",
    floor.c_str(), agents.size(), stands.size(), problem_path.c_str());
  const auto started = node->now();
  auto plan = planner->getPlan(domain->getDomain(), problem->getProblem());
  const double planning = (node->now() - started).seconds();

  bool planned = plan.has_value() && !plan->items.empty();
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
      node->get_logger(), "[mission] no policy for lifted: the planner returned none after "
      "%.1f s", planning);
    if (floor != "radio" && floor != "blind") {
      RCLCPP_ERROR(
        node->get_logger(), "[mission] mission failed: no policy on the %s floor", floor.c_str());
      rclcpp::shutdown();
      return 1;
    }
    if (floor == "radio" && !protocol.is_null()) {
      plan = found_protocol(mapping, reader, agents, stands, protocol);
      RCLCPP_INFO(
        node->get_logger(), "[mission] running the radio protocol instead: read the order, the "
        "%zu messages the planner found to reach E^%d among %zu robots, then lift",
        protocol.at(stands.at(0)).size(), protocol.at("depth").get<int>(), agents.size());
    } else if (floor == "radio") {
      plan = radio_protocol(mapping, reader, other, stands);
      RCLCPP_INFO(
        node->get_logger(), "[mission] running the radio protocol instead: read the order, four "
        "messages each one level deeper, then lift");
    } else {
      plan = blind_protocol(mapping, reader, other, stands);
      RCLCPP_INFO(
        node->get_logger(), "[mission] running the sight floor's policy without the sighting: "
        "both to their viewpoints, the order read, the signal, then lift");
    }
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
    RCLCPP_INFO(node->get_logger(), "[mission] mission complete: lifted");
  } else if (!planned && floor == "blind") {
    // Expected: the listener knows the stand, the announcer does not know that
    // it does, and so C does not hold and lift was refused for that reason.
    std::string verdict;
    for (const auto & s : stands) {
      const std::string job = "job_" + s;
      if (!holds(node, job)) {
        continue;
      }
      const std::string k1 = "(K " + other + " " + job + ")";
      const bool one = holds(node, k1);
      const bool two = holds(node, "(K " + reader + " " + k1 + ")");
      const bool common = holds(node, "(C (" + agents[0] + " " + agents[1] + ") " + job + ")");
      verdict = "the order named " + s + "; after the signal " + other + " knows it: " +
        (one ? "yes" : "no") + "; " + reader + " knows that " + other + " does: " +
        (two ? "yes" : "no") + "; C " + job + ": " + (common ? "yes" : "no");
      if (one && !two && !common) {
        RCLCPP_INFO(
          node->get_logger(), "[mission] mission complete: the executor refused lift, as the "
          "planner said it would; %s", verdict.c_str());
        rclcpp::shutdown();
        return 0;
      }
    }
    RCLCPP_ERROR(
      node->get_logger(), "[mission] mission failed before the signal: %s",
      verdict.empty() ? "the order was never read" : verdict.c_str());
    rclcpp::shutdown();
    return 1;
  } else if (!planned && !protocol.is_null()) {
    // As below, for the protocol tools/ladder.py found: E^k holds after its
    // last message, and C does not.
    const int depth = protocol.at("depth").get<int>();
    std::string verdict;
    for (const auto & s : stands) {
      const std::string job = "job_" + s;
      if (!holds(node, job)) {
        continue;
      }
      const bool deep = holds(node, everyone(depth, job, agents));
      const bool c = holds(node, common(agents, job));
      verdict = "the order named " + s + "; after the " +
        std::to_string(protocol.at(s).size()) + " messages E^" + std::to_string(depth) + " " +
        job + " " + (deep ? "holds" : "does not hold") + " among the " +
        std::to_string(agents.size()) + " robots, and C " + job + " " +
        (c ? "holds" : "does not hold");
      if (deep && !c) {
        // Not "as the planner said": with more robots its search may have
        // ended on its budget, and then it said nothing.
        RCLCPP_INFO(
          node->get_logger(), "[mission] mission complete: the executor refused lift, for want "
          "of C; %s", verdict.c_str());
        rclcpp::shutdown();
        return 0;
      }
    }
    RCLCPP_ERROR(
      node->get_logger(), "[mission] mission failed before the protocol ran out: %s",
      verdict.empty() ? "the order was never read" : verdict.c_str());
    rclcpp::shutdown();
    return 1;
  } else if (!planned) {
    // The run is the expected one only if the protocol got as far as it can:
    // all four messages applied, so E^4 holds, and C does not, so lift was
    // refused for the reason the planner gave and not for some other.
    std::string verdict;
    for (const auto & s : stands) {
      const std::string job = "job_" + s;
      if (!holds(node, job)) {
        continue;
      }
      const std::string deep =
        "(K " + agents[1] + " (K " + agents[0] + " (K " + agents[1] + " (K " + agents[0] + " " +
        job + "))))";
      const bool four = holds(node, deep);
      const bool common = holds(node, "(C (" + agents[0] + " " + agents[1] + ") " + job + ")");
      verdict = "the order named " + s + "; after the four messages the ack3 formula " +
        std::string(four ? "holds" : "does not hold") + ", and C " + job + " " +
        (common ? "holds" : "does not hold");
      if (four && !common) {
        RCLCPP_INFO(
          node->get_logger(), "[mission] mission complete: the executor refused lift, as the "
          "planner said it would; %s", verdict.c_str());
        rclcpp::shutdown();
        return 0;
      }
    }
    RCLCPP_ERROR(
      node->get_logger(), "[mission] mission failed before the protocol ran out: %s",
      verdict.empty() ? "the order was never read" : verdict.c_str());
    rclcpp::shutdown();
    return 1;
  } else {
    RCLCPP_ERROR(node->get_logger(), "[mission] mission failed");
  }
  rclcpp::shutdown();
  return succeeded ? 0 : 1;
}
