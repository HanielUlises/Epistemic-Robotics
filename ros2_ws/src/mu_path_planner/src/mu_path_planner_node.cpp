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

#include <sstream>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>

#include "mu_path_planner/mu_calculus.hpp"
#include "mu_path_planner/epistemic_state.hpp"
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
 *   /mu_planner/sensing     nav_msgs/Path  (the poses to sense from, in order)
 *
 * On each query: discretises the current OccupancyGrid, resolves the goal
 * zone, the safety constraint and the sensing cells against the Kripke
 * snapshot, runs mu_reach (or mu_reach_epistemic), and publishes the result.
 *
 * The node holds no planning logic of its own. Which cells are the goal is
 * decided in epistemic_state.hpp, the fixed point is taken in
 * mu_calculus.hpp, and both are testable without a graph.
 */
class MuPathPlannerNode : public rclcpp::Node {
public:
    explicit MuPathPlannerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions{})
    : Node("mu_path_planner", options)
    {
        // Where the line falls between free, occupied and undecided. The
        // defaults match nav2 and slam_toolbox; the band between them is a
        // cell that was seen without being settled, and it is not free.
        free_below_     = declare_parameter<int>("free_below", 25);
        occupied_above_ = declare_parameter<int>("occupied_above", 65);

        // How far a sensing action reaches, in cells. One means the agent has
        // to stand on or beside the cell it wants to resolve.
        sensor_range_cells_ =
            static_cast<uint32_t>(declare_parameter<int>("sensor_range_cells", 1));

        map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", rclcpp::QoS(1).transient_local(),
            [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
                on_map(msg);
            });

        state_sub_ = create_subscription<std_msgs::msg::String>(
            "/epistemic/state", rclcpp::QoS(1).transient_local(),
            [this](const std_msgs::msg::String::SharedPtr msg) {
                on_state(msg);
            });

        query_sub_ = create_subscription<epistemic_msgs::msg::MuPathQuery>(
            "/mu_planner/query", 10,
            [this](const epistemic_msgs::msg::MuPathQuery::SharedPtr msg) {
                on_query(msg);
            });

        path_pub_    = create_publisher<nav_msgs::msg::Path>("/mu_planner/path", 10);
        sensing_pub_ = create_publisher<nav_msgs::msg::Path>("/mu_planner/sensing", 10);

        // What the node is currently holding, latched. A query is answered
        // against one map and one snapshot, and which ones those are is not
        // otherwise visible from outside: a process that has just put a new
        // map on the wire cannot tell whether the answer it is about to read
        // was computed with that map or with the one before it. This says so,
        // and says it by content rather than by a sequence number nobody
        // else keeps.
        status_pub_ = create_publisher<std_msgs::msg::String>(
            "/mu_planner/status", rclcpp::QoS(1).transient_local());

        RCLCPP_INFO(get_logger(), "MuPathPlannerNode ready");
    }

private:
    // ------------------------------------------------------------------
    void on_map(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        map_meta_  = msg->info;
        map_frame_ = msg->header.frame_id;

        grid_.width      = msg->info.width;
        grid_.height     = msg->info.height;
        grid_.resolution = msg->info.resolution;
        grid_.origin_x   = msg->info.origin.position.x;
        grid_.origin_y   = msg->info.origin.position.y;

        const size_t n = static_cast<size_t>(msg->info.width) * msg->info.height;
        graph_.width  = msg->info.width;
        graph_.height = msg->info.height;
        graph_.obstacle.assign(n, true);
        cell_state_.assign(n, CellState::Unknown);

        for (size_t i = 0; i < n && i < msg->data.size(); ++i) {
            // -1 is a cell nobody observed; the band between the thresholds is
            // a cell observed without being settled. Neither is free, and the
            // fixed point excludes both: unknown is not free.
            const int8_t value = msg->data[i];
            if (value >= 0 && value < free_below_)      cell_state_[i] = CellState::Free;
            else if (value > occupied_above_)           cell_state_[i] = CellState::Occupied;
            else if (value >= 0)                        cell_state_[i] = CellState::Unknown;

            graph_.obstacle[i] = (cell_state_[i] != CellState::Free);
        }
        graph_.build_adjacency();

        // Poses and metric zone bounds were resolved against the previous
        // grid, so a snapshot held from before this map has to be reread.
        if (!last_state_json_.empty()) reparse_state();

        map_hash_ = content_hash(msg->data.data(), msg->data.size());
        publish_status();

        RCLCPP_INFO(get_logger(), "Map updated: %ux%u", graph_.width, graph_.height);
    }

    // ------------------------------------------------------------------
    void on_state(const std_msgs::msg::String::SharedPtr msg)
    {
        last_state_json_ = msg->data;
        if (grid_.width == 0) return;   // reparsed when the first map arrives
        reparse_state();
    }

    void reparse_state()
    {
        snapshot_ = parse_snapshot(last_state_json_, grid_);
        state_hash_ = content_hash(last_state_json_);
        publish_status();

        if (!snapshot_.ok) {
            RCLCPP_ERROR(get_logger(), "Unusable epistemic state: %s",
                         snapshot_.error.c_str());
            return;
        }
        RCLCPP_INFO(get_logger(),
            "Epistemic state: %zu worlds, %zu designated, %zu zones, %zu agents",
            snapshot_.worlds.size(), snapshot_.designated.size(),
            snapshot_.zones.size(), snapshot_.agents.size());
    }

    // ------------------------------------------------------------------
    void on_query(const epistemic_msgs::msg::MuPathQuery::SharedPtr msg)
    {
        if (graph_.width == 0) {
            RCLCPP_WARN(get_logger(), "No map yet — ignoring query");
            return;
        }
        if (!snapshot_.ok) {
            RCLCPP_WARN(get_logger(), "No epistemic state yet — ignoring query");
            return;
        }

        // The answer carries the stamp of the question, so that a consumer
        // can tell which query a path is the answer to. Without it a second
        // planner on the same graph -- one left running from an earlier
        // session is enough -- makes every subscriber read someone else's
        // answer as its own, and nothing in the message says otherwise.
        answering_ = msg->header.stamp;

        QuerySpec spec;
        spec.agent_id              = msg->agent_id;
        spec.goal_zone             = msg->goal_zone;
        spec.safety_formula_json   = msg->safety_formula_json;
        spec.require_epistemic_goal = msg->require_epistemic_goal;
        spec.sensor_range_cells    = sensor_range_cells_;

        const ResolvedQuery query =
            resolve_query(snapshot_, graph_, cell_state_, spec);

        if (!query.ok) {
            RCLCPP_ERROR(get_logger(), "Cannot answer the query: %s",
                         query.error.c_str());
            return;
        }

        if (query.start < graph_.obstacle.size() && graph_.obstacle[query.start]) {
            RCLCPP_WARN(get_logger(),
                "Agent %u stands on cell %u, which the map does not report free",
                spec.agent_id, query.start);
        }
        if (query.goal.empty()) {
            RCLCPP_WARN(get_logger(),
                "Zone '%s' covers no free cell in the designated worlds",
                spec.goal_zone.c_str());
        }

        std::vector<CellIdx> sensing_waypoints;
        MuReachResult result;

        if (msg->require_epistemic_goal) {
            if (query.sensing.empty()) {
                RCLCPP_WARN(get_logger(),
                    "Zone '%s': nothing to sense and nothing already known — "
                    "the agent cannot come to know it arrived",
                    spec.goal_zone.c_str());
            }
            auto epistemic = mu_reach_epistemic(
                graph_, query.goal, query.safe, query.sensing, query.start);
            sensing_waypoints = std::move(epistemic.sensing_waypoints);
            result.winning_region = std::move(epistemic.winning_region);
            result.path           = std::move(epistemic.path);
            result.iterations     = epistemic.iterations;
        } else {
            result = mu_reach(graph_, query.goal, query.safe, query.start);
        }

        RCLCPP_INFO(get_logger(),
            "mu_reach agent=%u zone='%s' epistemic=%d: goal=%zu known=%zu "
            "disputed=%zu sensing=%zu safe=%zu | region=%zu path=%zu iters=%u",
            spec.agent_id, spec.goal_zone.c_str(),
            static_cast<int>(msg->require_epistemic_goal),
            query.goal.size(), query.known_goal.size(), query.disputed.size(),
            query.sensing.size(), query.safe.size(),
            result.winning_region.size(), result.path.size(), result.iterations);

        if (result.path.empty()) {
            RCLCPP_WARN(get_logger(),
                "No route: cell %u is outside the winning region", query.start);
        }

        publish_path(path_pub_, result.path);
        publish_path(sensing_pub_, sensing_waypoints);
    }

    // ------------------------------------------------------------------
    void publish_status()
    {
        std::ostringstream out;
        out << "{\"map_hash\": " << map_hash_
            << ", \"state_hash\": " << state_hash_
            << ", \"width\": " << graph_.width
            << ", \"height\": " << graph_.height
            << ", \"state_ok\": " << (snapshot_.ok ? "true" : "false")
            << ", \"worlds\": " << snapshot_.worlds.size()
            << ", \"designated\": " << snapshot_.designated.size()
            << ", \"zones\": " << snapshot_.zones.size()
            << ", \"agents\": " << snapshot_.agents.size()
            << "}";

        std_msgs::msg::String message;
        message.data = out.str();
        status_pub_->publish(message);
    }

    // ------------------------------------------------------------------
    void publish_path(const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr& pub,
                      const std::vector<CellIdx>& cell_path)
    {
        nav_msgs::msg::Path ros_path;
        ros_path.header.stamp    = answering_;
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

        pub->publish(ros_path);
    }

    // ------------------------------------------------------------------
    OccupancyGraph graph_;
    GridInfo grid_;
    std::vector<CellState> cell_state_;
    nav_msgs::msg::MapMetaData map_meta_;
    std::string map_frame_{"map"};

    std::string last_state_json_;
    EpistemicSnapshot snapshot_;

    int free_below_{25};
    int occupied_above_{65};
    uint32_t sensor_range_cells_{1};

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr  map_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr          state_sub_;
    rclcpp::Subscription<epistemic_msgs::msg::MuPathQuery>::SharedPtr query_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr               path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr               sensing_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr             status_pub_;

    uint64_t map_hash_{0};
    uint64_t state_hash_{0};
    builtin_interfaces::msg::Time answering_;
};

}  // namespace mu_path_planner

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<mu_path_planner::MuPathPlannerNode>());
    rclcpp::shutdown();
    return 0;
}
