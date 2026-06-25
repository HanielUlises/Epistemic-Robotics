#include "epistemic_state/epistemic_world_manager.hpp"
#include <std_msgs/msg/string.hpp>

namespace epistemic_state {

EpistemicWorldManager::EpistemicWorldManager(const rclcpp::NodeOptions& options)
: Node("epistemic_world_manager", options)
{
    declare_parameter("num_agents", 1);
    num_agents_ = static_cast<size_t>(get_parameter("num_agents").as_int());

    event_sub_ = create_subscription<epistemic_msgs::msg::EpistemicEvent>(
        "/epistemic/events", 10,
        [this](const epistemic_msgs::msg::EpistemicEvent::SharedPtr msg) {
            on_epistemic_event(msg);
        });

    state_pub_ = create_publisher<std_msgs::msg::String>("/epistemic/state", 10);

    RCLCPP_INFO(get_logger(),
        "EpistemicWorldManager ready — %zu agent(s)", num_agents_);
}

void EpistemicWorldManager::on_epistemic_event(
    const epistemic_msgs::msg::EpistemicEvent::SharedPtr msg)
{
    // TODO (TT-II): deserialise msg->formula_json via plank,
    // build the DEL event model, run product_update(), apply
    // bisim_contract(), and publish the updated model as JSON.
    //
    // For now, log the incoming event so the pipeline can be
    // smoke-tested end-to-end without the planner linked.
    RCLCPP_INFO(get_logger(),
        "Event from agent %u | type=%u | confidence=%.2f | formula=%s",
        msg->agent_id, msg->event_type,
        msg->confidence, msg->formula_json.c_str());

    std_msgs::msg::String out;
    out.data = "{\"status\":\"stub\",\"agent\":" +
               std::to_string(msg->agent_id) + "}";
    state_pub_->publish(out);
}

}  // namespace epistemic_state

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<epistemic_state::EpistemicWorldManager>());
    rclcpp::shutdown();
    return 0;
}
