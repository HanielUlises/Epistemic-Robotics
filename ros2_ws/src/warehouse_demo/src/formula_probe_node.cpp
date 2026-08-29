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
// Asks the epistemic state, twice a second, whether each of several formulas
// holds -- and writes down what it said.
//
// This node decides nothing and is dispatched by nobody. It is an instrument:
// the run is going to be watched, and what a viewer needs to see is not that
// the robot reached the bay but the moment one formula becomes true while
// another over the same model does not. That moment has no picture of its own.
// It is a change in a Kripke model, and the only way to put a timestamp on it
// is to keep asking.
//
// The three formulas it is given in this demonstration are
//
//     (Kw r1 pallet-at_bay2)                  r1 knows whether
//     (K r2 (Kw r1 pallet-at_bay2))           r2 knows that r1 knows whether
//     (Kw r2 pallet-at_bay2)                  r2 knows whether
//
// and they are asked of the *same* state in the *same* run, whichever of them
// the policy was planned for. That is what makes the log evidence rather than
// illustration: the goal the planner was given cannot be what decides the
// answers, because two of the three formulas were never its goal.
//
// The middle one is nested, and nothing here had to be taught to handle that.
// `CheckFormula`'s parser is recursive over K, Kw, C and the connectives; the
// depth-one formulas the warehouse ever asked it were a use of it, not its
// limit. What had never been written down was a goal that needed the depth.

#include <chrono>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "plansys2_epistemic_executor/EpistemicStateClient.hpp"

using namespace std::chrono_literals;

class FormulaProbe : public rclcpp::Node
{
public:
  FormulaProbe()
  : Node("formula_probe")
  {
    // The formulas, and a short name for each so the CSV has readable columns.
    // Given as two parallel arrays rather than one map because ROS parameters
    // have no map type and the alternative -- one parameter per formula -- is
    // a node that has to be edited to watch something else.
    declare_parameter<std::vector<std::string>>("formulas", std::vector<std::string>{});
    declare_parameter<std::vector<std::string>>("labels", std::vector<std::string>{});
    declare_parameter<double>("period", 0.5);
    declare_parameter<std::string>("csv", "");
    // For the header of the file: which goal this run was planned for. The
    // probe asks all of the formulas either way, and writing down which one
    // was the goal is what lets two runs be read against each other.
    declare_parameter<std::string>("goal", "");

    formulas_ = get_parameter("formulas").as_string_array();
    labels_ = get_parameter("labels").as_string_array();
    if (labels_.size() != formulas_.size()) {
      labels_ = formulas_;
    }
    held_.assign(formulas_.size(), -1);

    state_ = std::make_shared<plansys2::EpistemicStateClient>("formula_probe_state_client");

    // What the mission is doing, so a row of the log can be placed in the run
    // without anyone having to line up two clocks. Transient local, because
    // the phase is published on change and this node starts when it starts.
    phase_ = create_subscription<std_msgs::msg::String>(
      "/warehouse/phase", rclcpp::QoS(1).transient_local(),
      [this](std_msgs::msg::String::SharedPtr message) {phase_text_ = message->data;});

    open_csv();

    const auto period = std::chrono::duration<double>(get_parameter("period").as_double());
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period), [this] {tick();});

    RCLCPP_INFO(
      get_logger(), "watching %zu formulas every %.2f s",
      formulas_.size(), get_parameter("period").as_double());
    for (std::size_t i = 0; i < formulas_.size(); ++i) {
      RCLCPP_INFO(get_logger(), "  %-10s %s", labels_[i].c_str(), formulas_[i].c_str());
    }
  }

private:
  void open_csv()
  {
    const auto path = get_parameter("csv").as_string();
    if (path.empty()) {
      return;
    }
    csv_.open(path, std::ios::out | std::ios::trunc);
    if (!csv_) {
      RCLCPP_ERROR(get_logger(), "cannot write %s; logging to the console only", path.c_str());
      return;
    }
    csv_ << "# goal planned for: " << get_parameter("goal").as_string() << "\n";
    for (std::size_t i = 0; i < formulas_.size(); ++i) {
      csv_ << "# " << labels_[i] << " = " << formulas_[i] << "\n";
    }
    csv_ << "wall_s,sim_s,phase";
    for (const auto & label : labels_) {
      csv_ << "," << label;
    }
    csv_ << "\n";
    csv_.flush();
  }

  /// What one answer is worth writing. A formula the state could not be asked
  /// is not a formula that failed to hold, and the two must not share a cell:
  /// the whole result of this run is that one formula is true while another is
  /// false at the same instant, and an unreachable service reported as `false`
  /// would manufacture exactly that.
  static const char * cell(const plansys2::EpistemicStateClient::Answer & answer)
  {
    if (!answer.answered) {return "no-answer";}
    if (!answer.success) {return "error";}
    return answer.holds ? "TRUE" : "FALSE";
  }

  void tick()
  {
    const double wall = std::chrono::duration<double>(
      std::chrono::steady_clock::now().time_since_epoch()).count() - started_;
    const double sim = now().seconds();

    std::string row;
    for (std::size_t i = 0; i < formulas_.size(); ++i) {
      const auto answer = state_->check_formula(formulas_[i], 400ms);
      const char * text = cell(answer);
      row += ",";
      row += text;

      // The console gets the transitions and nothing else. A line every half
      // second for the length of a warehouse run is not a log anybody reads;
      // the moment a formula changes is the only thing in it.
      const int value = answer.answered && answer.success ? (answer.holds ? 1 : 0) : -1;
      if (value != held_[i]) {
        if (value >= 0) {
          RCLCPP_INFO(
            get_logger(), "[t=%7.1f] %-10s %-5s   %s",
            wall, labels_[i].c_str(), value ? "TRUE" : "FALSE", formulas_[i].c_str());
        } else if (held_[i] >= 0) {
          RCLCPP_WARN(
            get_logger(), "[t=%7.1f] %-10s unanswerable: %s",
            wall, labels_[i].c_str(), answer.error.c_str());
        }
        held_[i] = value;
      }
    }

    if (csv_) {
      csv_ << wall << "," << sim << ",\"" << phase_text_ << "\"" << row << "\n";
      csv_.flush();
    }
  }

  std::shared_ptr<plansys2::EpistemicStateClient> state_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr phase_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<std::string> formulas_, labels_;
  std::vector<int> held_;
  std::string phase_text_;
  std::ofstream csv_;
  double started_{std::chrono::duration<double>(
      std::chrono::steady_clock::now().time_since_epoch()).count()};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FormulaProbe>());
  rclcpp::shutdown();
  return 0;
}
