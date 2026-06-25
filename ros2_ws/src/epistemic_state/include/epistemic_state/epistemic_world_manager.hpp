#pragma once

#include <rclcpp/rclcpp.hpp>
#include <epistemic_msgs/msg/epistemic_event.hpp>

// Forward-declare the planner types so this node can hold an EpistemicState
// without pulling in the full planner build (eplansys is a separate package).
// In TT-II this will be replaced by a proper eplansys client interface.
struct EpistemicState;

namespace epistemic_state {

/**
 * EpistemicWorldManager
 *
 * Maintains the shared multi-agent Kripke model M = (W, R_1..n, V, W*).
 * Subscribes to /epistemic/events and applies the DEL product update
 * M ⊗ a for each incoming event, keeping W* (designated worlds) consistent.
 *
 * Publishes a JSON-serialised snapshot of the current model on
 * /epistemic/state after every update, so other nodes (mu_path_planner,
 * eplansys dispatcher) can consume it without sharing memory.
 *
 * Latency target: < 500 ms per update (RNF-01 from TT-I spec).
 */
class EpistemicWorldManager : public rclcpp::Node {
public:
    explicit EpistemicWorldManager(const rclcpp::NodeOptions& options = rclcpp::NodeOptions{});

private:
    void on_epistemic_event(const epistemic_msgs::msg::EpistemicEvent::SharedPtr msg);

    rclcpp::Subscription<epistemic_msgs::msg::EpistemicEvent>::SharedPtr event_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr                  state_pub_;

    // Number of agents in the current mission (set as ROS param "num_agents")
    size_t num_agents_{1};
};

}  // namespace epistemic_state
