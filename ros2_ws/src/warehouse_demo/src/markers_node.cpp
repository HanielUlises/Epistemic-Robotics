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
// What the robot knows, drawn over the map it is building.
//
// A map shows where a robot has been and what it bumped into. It cannot show
// that two bays are still in dispute, or that the model designates two worlds
// and will designate one after the next look. That is the part of this system
// worth watching, and it has no picture unless something draws it.
//
// Everything here is read, not decided: the state comes from ePlanSys's own
// epistemic state node, latched on epistemic_state/state, and the action being
// run comes from the executor's feedback. This node adds nothing to the
// system's knowledge -- if it stopped, the mission would be unchanged.

#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "plansys2_epistemic_executor/EpistemicStateClient.hpp"
#include "plansys2_msgs/msg/action_execution_info.hpp"
#include "warehouse_scenario/warehouse.hpp"

using namespace std::chrono_literals;
using namespace warehouse_scenario;

namespace
{

/// The zones are written in the AWS world's frame and the map frame comes up
/// on the world origin, so a layout box is already a map box. See the same
/// function in drive_action_node, which is where getting this wrong shows.
Box in_map(const Box & layout)
{
  return layout;
}

/// The mission step, in words, from the action the executor is running.
///
/// Modelled on demo/demo_driver.py, which narrates its nine steps rather than
/// reporting its internals. The difference matters on a recording: "no
/// measured route yet" and "2 worlds still possible" are true and tell a
/// viewer nothing about what the robot is doing or why. What they want to know
/// is which part of the mission this is.
std::string narrate(const std::string & running)
{
  if (running.empty()) {return "starting up";}

  std::vector<std::string> words;
  std::istringstream stream(running);
  for (std::string word; stream >> word; ) {words.push_back(word);}
  if (words.empty()) {return running;}

  const std::string & verb = words.front();
  const std::string & last = words.back();

  // Short. An rviz text marker draws in 3D and long lines come out stretched
  // and spindly -- legible on a still, not in a recording that has been sped
  // up and scaled down. Every line here is kept under about forty characters,
  // which is what makes the difference between text and decoration.
  if (verb == "look_into") {return "LOOKING into " + last;}
  if (verb == "pick_up") {return "PICKING UP in " + last;}
  if (verb == "drop_off") {return "UNLOADING at " + last;}
  if (verb == "goto_zone") {return "DRIVING to " + last;}
  return running;
}

/// One value out of the epistemic state's JSON. Reading it with find is enough
/// for a caption and keeps this node free of a JSON dependency: the fields are
/// numbers and short strings, and a caption that cannot be parsed is left out
/// rather than guessed at.
std::string field(const std::string & json, const std::string & key)
{
  const auto at = json.find("\"" + key + "\"");
  if (at == std::string::npos) {return "";}
  auto colon = json.find(':', at);
  if (colon == std::string::npos) {return "";}
  ++colon;
  while (colon < json.size() && (json[colon] == ' ' || json[colon] == '"')) {++colon;}
  auto end = colon;
  while (end < json.size() && json[end] != ',' && json[end] != '}' && json[end] != '"') {++end;}
  return json.substr(colon, end - colon);
}

}  // namespace

class Markers : public rclcpp::Node
{
public:
  Markers()
  : Node("warehouse_markers")
  {
    markers_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/warehouse/markers", rclcpp::QoS(1).transient_local());

    state_ = create_subscription<std_msgs::msg::String>(
      "epistemic_state/state", rclcpp::QoS(1).transient_local(),
      [this](const std_msgs::msg::String::SharedPtr msg) {state_json_ = msg->data;});

    // /action_execution_info, not /actions_hub.
    //
    // Both exist. /actions_hub carries two different message types -- the
    // executor's ActionExecution request protocol and ActionExecutionInfo --
    // and a subscription typed to one of them silently receives none of the
    // other. The caption stayed on its start-up line for entire runs because
    // of it: no error, no warning, just an action name that never arrived.
    // /action_execution_info carries only the status reports.
    actions_ = create_subscription<plansys2_msgs::msg::ActionExecutionInfo>(
      "/action_execution_info", rclcpp::QoS(100),
      [this](const plansys2_msgs::msg::ActionExecutionInfo::SharedPtr msg) {
        if (msg->status == plansys2_msgs::msg::ActionExecutionInfo::EXECUTING) {
          std::string named = msg->action;
          for (const auto & argument : msg->arguments) {named += " " + argument;}
          if (named != running_) {
            ++step_;   // demo/ numbers its steps; a viewer needs to know
            running_ = named;  // whether the picture moved on
          }
        }
      });

    activity_ = create_subscription<std_msgs::msg::String>(
      "/warehouse/activity", rclcpp::QoS(1).transient_local(),
      [this](const std_msgs::msg::String::SharedPtr msg) {doing_ = msg->data;});

    // What the mission is doing before it has an action to run: coming up,
    // seeding the problem, waiting on the policy. Without it the caption has
    // nothing to say for the first minute of a recording.
    phase_ = create_subscription<std_msgs::msg::String>(
      "/warehouse/phase", rclcpp::QoS(1).transient_local(),
      [this](const std_msgs::msg::String::SharedPtr msg) {phase_text_ = msg->data;});

    // The epistemic state itself, not the drive's geometry snapshot.
    //
    // /epistemic/state is published by drive_action and says "one world, one
    // designated" always -- it carries zone boxes and a pose, and nothing
    // about the pallet. Counting its worlds put "1 world still possible" on
    // screen from the first frame and claimed the robot knew which bay before
    // it had left the depot. What actually holds the question is ePlanSys's
    // epistemic state, and this asks it the same way look_action does.
    knowledge_ = std::make_shared<plansys2::EpistemicStateClient>(
      "markers_state_client");

    buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);

    timer_ = create_wall_timer(200ms, [this] {publish();});
  }

private:
  visualization_msgs::msg::Marker base(int id, int type)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = now();
    marker.ns = "warehouse";
    marker.id = id;
    marker.type = type;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    return marker;
  }

  void zone(visualization_msgs::msg::MarkerArray & array, int id, const Box & box,
    double r, double g, double b, double a, const std::string & label)
  {
    auto patch = base(id, visualization_msgs::msg::Marker::CUBE);
    patch.pose.position.x = (box.min_x + box.max_x) / 2.0;
    patch.pose.position.y = (box.min_y + box.max_y) / 2.0;
    patch.pose.position.z = 0.01;
    patch.scale.x = box.max_x - box.min_x;
    patch.scale.y = box.max_y - box.min_y;
    patch.scale.z = 0.02;
    patch.color.r = r; patch.color.g = g; patch.color.b = b; patch.color.a = a;
    array.markers.push_back(patch);

    auto text = base(id + 100, visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
    text.pose.position.x = (box.min_x + box.max_x) / 2.0;
    text.pose.position.y = box.max_y + 0.4;
    text.pose.position.z = 0.4;
    text.scale.z = 0.4;
    text.color.r = 1.0; text.color.g = 1.0; text.color.b = 1.0; text.color.a = 0.9;
    text.text = label;
    array.markers.push_back(text);
  }

  /// Does r1 know which bay the pallet is in? Asked of ePlanSys, once a
  /// second: a formula check is a service round trip and the caption does not
  /// need it at the rate the markers redraw.
  void refresh_knowledge()
  {
    if ((now() - asked_at_).seconds() < 1.0) {return;}
    asked_at_ = now();

    // The goal, in the language the domain states it in. Asked once: it does
    // not change while a policy runs, and it is the thing every step is for.
    if (goal_.empty()) {
      const auto answer = knowledge_->get_goal();
      if (answer.answered && answer.success) {goal_ = answer.goal;}
    }

    for (const char * formula :
      {"(Kw r1 pallet-at_bay2)", "(Kw r1 pallet-at_bay3)"})
    {
      const auto answer = knowledge_->check_formula(formula);
      if (answer.answered && answer.success && answer.holds) {
        settled_ = true;
        return;
      }
    }
  }

  void publish()
  {
    refresh_knowledge();
    visualization_msgs::msg::MarkerArray array;

    zone(array, 1, in_map(kDockNorth), 0.12, 0.55, 0.35, 0.5, "dock_north");
    zone(array, 2, in_map(kDockSouth), 0.12, 0.55, 0.35, 0.25, "dock_south");

    // Both bays are drawn while the question is open, because that is what
    // the model says: one of them holds the pallet and nothing yet says which.
    // The state reports how many worlds it designates, and that number falling
    // to one is the observation having landed.
    // *Designated* worlds, not worlds: the model keeps both worlds after an
    // observation, and what changes is how many of them the agent still counts
    // as possible. That number falling to one is the observation having
    // landed, and it is the number worth putting on screen.
    const bool settled = settled_;
    // Named the way demo/ names an unobserved room: the label says what is
    // not known about it, not just what it is called.
    zone(array, 3, in_map(kBayAisle2), 0.85, 0.62, 0.10, settled ? 0.7 : 0.35,
      settled ? "bay2" : "bay2 (pallet?)");
    zone(array, 4, in_map(kBayAisle3), 0.85, 0.62, 0.10, settled ? 0.7 : 0.35,
      settled ? "bay3" : "bay3 (pallet?)");

    // No caption marker here on purpose.
    //
    // An rviz TEXT_VIEW_FACING marker draws text as geometry in the scene: it
    // scales with the camera, spaces words oddly, and thins out to something
    // unreadable once a recording is scaled down. Several attempts at making
    // it legible failed for that reason. The narration is burned into the
    // video afterwards instead, by scenarios/warehouse/tools/caption_video.py,
    // where it is a real font at a real pixel size. The zone labels stay --
    // they are single short words and render fine.

    markers_->publish(array);
  }

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_;
  rclcpp::Subscription<plansys2_msgs::msg::ActionExecutionInfo>::SharedPtr actions_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string state_json_;
  std::string doing_;
  std::shared_ptr<plansys2::EpistemicStateClient> knowledge_;
  rclcpp::Time asked_at_{0, 0, RCL_ROS_TIME};
  bool settled_{false};
  std::string goal_;
  int step_{0};
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr activity_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr phase_;
  std::string phase_text_;
  std::shared_ptr<tf2_ros::Buffer> buffer_;
  std::shared_ptr<tf2_ros::TransformListener> listener_;
  std::string running_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Markers>());
  rclcpp::shutdown();
  return 0;
}
