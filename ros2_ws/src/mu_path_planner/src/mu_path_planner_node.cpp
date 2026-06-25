#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>

#include "mu_path_planner/mu_calculus.hpp"
#include "epistemic_msgs/msg/mu_path_query.hpp"

namespace mu_path_planner {

/**
 * MuPathPlannerNode
 *
 * Subscribes to:
 *   /map                    nav_msgs/OccupancyGrid  (from SLAM Toolbox)
 *   /mu_planner/query       epistemic_msgs/MuPathQuery
 *   /epistemic/state        std_msgs/String (JSON Kripke snapshot)
 *
 * Publishes:
 *   /mu_planner/path        nav_msgs/Path
 *
 * On each query: discretises the current OccupancyGrid, resolves the goal
 * zone to a set of cells via the Kripke snapshot, runs mu_reach (or
 * mu_reach_epistemic), and publishes the resulting path.
 */
class MuPathPlannerNode : public rclcpp::Node {
public:
    explicit MuPathPlannerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions{})
    : Node("mu_path_planner", options)
    {
        map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", rclcpp::QoS(1).transient_local(),
            [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
                on_map(msg);
            });

        state_sub_ = create_subscription<std_msgs::msg::String>(
            "/epistemic/state", 10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                last_state_json_ = msg->data;
            });

        query_sub_ = create_subscription<epistemic_msgs::msg::MuPathQuery>(
            "/mu_planner/query", 10,
            [this](const epistemic_msgs::msg::MuPathQuery::SharedPtr msg) {
                on_query(msg);
            });

        path_pub_ = create_publisher<nav_msgs::msg::Path>("/mu_planner/path", 10);

        RCLCPP_INFO(get_logger(), "MuPathPlannerNode ready");
    }

private:
    // ------------------------------------------------------------------
    void on_map(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        map_meta_  = msg->info;
        map_frame_ = msg->header.frame_id;

        const size_t n = msg->info.width * msg->info.height;
        graph_.width  = msg->info.width;
        graph_.height = msg->info.height;
        graph_.obstacle.resize(n);

        for (size_t i = 0; i < n; ++i) {
            // OccupancyGrid: -1 = unknown, 0 = free, 100 = occupied
            // Treat unknown as obstacle (conservative for epistemic planning)
            graph_.obstacle[i] = (msg->data[i] != 0);
        }
        graph_.build_adjacency();

        RCLCPP_INFO(get_logger(), "Map updated: %ux%u", graph_.width, graph_.height);
    }

    // ------------------------------------------------------------------
    void on_query(const epistemic_msgs::msg::MuPathQuery::SharedPtr msg)
    {
        if (graph_.width == 0) {
            RCLCPP_WARN(get_logger(), "No map yet — ignoring query");
            return;
        }

        // TODO (TT-II): parse last_state_json_ to derive goal_set and
        // sensing_cells from the Kripke model for msg->goal_zone.
        // For now, use a placeholder: last row of free cells as goal.
        std::unordered_set<CellIdx> goal_set;
        std::unordered_set<CellIdx> safe_set;

        const size_t n = static_cast<size_t>(graph_.width) * graph_.height;
        for (CellIdx i = 0; i < static_cast<CellIdx>(n); ++i) {
            if (!graph_.obstacle[i]) {
                safe_set.insert(i);
                // Placeholder goal: rightmost column
                if (i % graph_.width == graph_.width - 1)
                    goal_set.insert(i);
            }
        }

        CellIdx start = graph_.cell(graph_.height / 2, 0);

        MuReachResult result;
        if (msg->require_epistemic_goal) {
            // sensing_cells placeholder: cells adjacent to goal
            std::unordered_set<CellIdx> sensing;
            for (CellIdx g : goal_set)
                for (CellIdx nb : graph_.neighbours[g])
                    sensing.insert(nb);
            result = mu_reach_epistemic(graph_, goal_set, safe_set, sensing, start);
        } else {
            result = mu_reach(graph_, goal_set, safe_set, start);
        }

        RCLCPP_INFO(get_logger(),
            "mu_reach: %zu cells in winning region, path length=%zu, iters=%u",
            result.winning_region.size(), result.path.size(), result.iterations);

        publish_path(result.path, msg->header);
    }

    // ------------------------------------------------------------------
    void publish_path(const std::vector<CellIdx>& cell_path,
                      const std_msgs::msg::Header& query_header)
    {
        nav_msgs::msg::Path ros_path;
        ros_path.header.stamp    = get_clock()->now();
        ros_path.header.frame_id = map_frame_;

        const double res = map_meta_.resolution;
        const double ox  = map_meta_.origin.position.x;
        const double oy  = map_meta_.origin.position.y;

        for (CellIdx idx : cell_path) {
            uint32_t col = idx % graph_.width;
            uint32_t row = idx / graph_.width;

            geometry_msgs::msg::PoseStamped pose;
            pose.header = ros_path.header;
            pose.pose.position.x = ox + (col + 0.5) * res;
            pose.pose.position.y = oy + (row + 0.5) * res;
            pose.pose.orientation.w = 1.0;
            ros_path.poses.push_back(pose);
        }

        path_pub_->publish(ros_path);
    }

    // ------------------------------------------------------------------
    OccupancyGraph graph_;
    nav_msgs::msg::MapMetaData map_meta_;
    std::string map_frame_{"map"};
    std::string last_state_json_;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr  map_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr          state_sub_;
    rclcpp::Subscription<epistemic_msgs::msg::MuPathQuery>::SharedPtr query_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr               path_pub_;
};

}  // namespace mu_path_planner

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<mu_path_planner::MuPathPlannerNode>());
    rclcpp::shutdown();
    return 0;
}
