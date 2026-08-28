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
// `goto_zone`: the one thing a planning stack cannot supply, which is an
// action that moves the robot.
//
// The route is not this node's own work either. It asks mu_path_planner for
// one, which takes the least fixed point over the map slam_toolbox is
// building, and follows what comes back. That division is the point of having
// both planners: ePlanSys decides which zone to be in and what has to be known
// there, and the mu-calculus decides how to cross a floor that is still half
// unobserved -- refusing, correctly, to drive through a corridor nobody has
// looked down.
//
// The zone geometry this node publishes on /epistemic/state is geometry only:
// one world, four boxes, the robot's live pose. What is *not* known -- which
// bay holds the pallet -- lives in the epistemic state node of ePlanSys, where
// the policy and perception both reach it. Putting it here as well would be
// two models of one thing, and they would drift.

#include <algorithm>
#include <cmath>
#include <memory>
#include <deque>
#include <tuple>
#include <sstream>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "epistemic_msgs/msg/mu_path_query.hpp"
#include "plansys2_executor/ActionExecutorClient.hpp"
#include "warehouse_scenario/warehouse.hpp"

using namespace warehouse_scenario;

namespace
{

constexpr double kArrived = 0.35;     // m, close enough to the zone
constexpr double kRequery = 4.0;      // s between asking for a fresh route

/// How far the chosen target must move before Nav2 is given a new goal.
///
/// The mu-calculus is re-asked every few seconds and its route may end a cell
/// or two from where it ended last time. Forwarding each of those to Nav2
/// cancels and restarts the navigation, and a robot that is continually
/// restarted never gets anywhere.
constexpr double kGoalMoved = 0.4;

constexpr int kInflationCells = 6;

/// How long a chosen frontier is committed to, in seconds.
///
/// Recomputing it on every requery makes the robot dither: it drives a metre,
/// the nearest-to-the-goal reachable cell is now somewhere slightly behind it,
/// and it turns around to go there. Committing to a target until it is reached
/// is what turns that into travel.
constexpr double kFrontierHold = 25.0;

/// How long a visited frontier stays spent, in seconds.
///
/// Not for the whole drive. A frontier is spent because going there did not
/// pay *on the map as it stood*, and the map does not stand still: the trip
/// itself measured floor, and what was a dead end from one side may be the
/// obvious way in once the ground beside it is known. Blacklisting for good
/// steadily eats the candidate set until the only frontiers left are the far
/// ones, and the robot walks away from where it is going.
constexpr double kSpentFor = 90.0;

/// How close to a spent frontier still counts as the same place.
constexpr double kSpentRadius = 0.5;

/// The warehouse zones in the map frame, which is the world frame.
///
/// The burger's odometry is published in the world's coordinates rather than
/// relative to wherever it was spawned, so slam_toolbox's map frame comes up
/// on the world origin and not on the robot. A zone box written in the AWS
/// world's frame -- which is what warehouse_scenario holds, and what the
/// snapshots quote -- is therefore already a map box, and the right amount to
/// shift it by is none.
///
/// This function is kept rather than inlined because getting it wrong is not
/// visible as a crash. Shift the zones by the robot's start and every one of
/// them lands a few metres off, most of them inside a rack; the planner then
/// correctly reports no route to a goal that is inside an obstacle, the drive
/// gives up and backs out, and what you see is a robot turning on the spot in
/// an empty aisle. `check_frames` below is what says so out loud.
Box in_map(const Box & layout)
{
  return layout;
}

/// A coordinate with two decimals, for a caption rather than a log.
std::string trimmed(double value)
{
  std::ostringstream out;
  out.setf(std::ios::fixed);
  out.precision(2);
  out << value;
  return out.str();
}

Box zone_named(const std::string & name)
{
  if (name == "bay2") {return in_map(kBayAisle2);}
  if (name == "bay3") {return in_map(kBayAisle3);}
  if (name == "dock_north") {return in_map(kDockNorth);}
  if (name == "dock_south") {return in_map(kDockSouth);}
  // The service lane in front of the racks, and the west corridor: the two
  // zones the domain crosses that are floor rather than a place to stop.
  if (name == "lane") {
    return in_map(Box{kServiceLaneX - 0.6, -8.8, kServiceLaneX + 0.6, -7.4});
  }
  if (name == "corridor") {return in_map(Box{-4.2, -2.0, -2.8, -0.6});}
  // "frontier" is not a zone of the domain: it is wherever the drive has
  // decided to go and look next, and the node holds it rather than this table.
  // Anything else is where the robot started, which is the map origin.
  return Box{-0.7, -0.7, 0.7, 0.7};
}

std::string bounds_json(const Box & box)
{
  std::ostringstream out;
  out << "{\"bounds\": {\"min_x\": " << box.min_x << ", \"min_y\": " << box.min_y
      << ", \"max_x\": " << box.max_x << ", \"max_y\": " << box.max_y << "}}";
  return out.str();
}

}  // namespace

class DriveAction : public plansys2::ActionExecutorClient
{
public:
  DriveAction()
  : plansys2::ActionExecutorClient("drive_action")
  {
    declare_parameter<double>("timeout", 240.0);

    // Which robot this node drives. It is only used to sign what the node
    // says: which drives it may accept at all is `specialized_arguments`,
    // which the executor matches positionally against the action's own
    // arguments, so a node started for r2 never claims r1's drive.
    declare_parameter<std::string>("robot", "r1");

    // See look_action_node: `map` is shared by the fleet, the base link is
    // not.
    declare_parameter<std::string>("base_frame", "base_footprint");

    // Nav2 does the driving. This node never writes to cmd_vel: two things
    // publishing velocities is how a robot ends up doing neither thing.
    //
    // Every name below is relative, and that is what makes a fleet possible.
    // Launched inside the namespace of one robot they resolve to that robot's
    // own Nav2, its own SLAM map, and its own route planner; launched without
    // one they resolve exactly as the absolute names they replaced. A robot
    // reasoning over the fleet's shared map would be the wrong thing anyway:
    // sensing here is semi-private, and what r2 has measured is not something
    // r1 knows.
    nav_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
    // A plain sentence about what the drive is doing right now, for the
    // caption in rviz. Watching a robot cross a warehouse tells you nothing
    // about why it chose that direction; this is the why.
    //
    // This one *is* absolute: the caption is the fleet's, not a robot's, and
    // three of these interleaving on one topic is what it wants to show. Each
    // line is signed, because an unattributed sentence about a warehouse with
    // three robots in it says nothing.
    activity_ = create_publisher<std_msgs::msg::String>(
      "/warehouse/activity", rclcpp::QoS(1).transient_local());
    state_ = create_publisher<std_msgs::msg::String>(
      "epistemic/state", rclcpp::QoS(1).transient_local());
    query_ = create_publisher<epistemic_msgs::msg::MuPathQuery>("mu_planner/query", 10);

    map_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "map", rclcpp::QoS(1).transient_local(),
      [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {grid_ = msg;});
    path_ = create_subscription<nav_msgs::msg::Path>(
      "mu_planner/path", 10,
      [this](const nav_msgs::msg::Path::SharedPtr msg) {on_path(msg);});

    buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);
  }

private:
  // -- inputs ---------------------------------------------------------------

  void on_path(const nav_msgs::msg::Path::SharedPtr msg)
  {
    if (msg->poses.empty()) {
      empty_answer_ = true;
      return;
    }
    empty_answer_ = false;
    smooth(*msg);
  }

  bool pose()
  {
    try {
      const auto t = buffer_->lookupTransform(
        "map", get_parameter("base_frame").as_string(), tf2::TimePointZero);
      x_ = t.transform.translation.x;
      y_ = t.transform.translation.y;
      check_frames();
      const auto & q = t.transform.rotation;
      yaw_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
      return true;
    } catch (const std::exception &) {
      return false;
    }
  }

  // -- the route ------------------------------------------------------------

  bool free_at(double x, double y) const
  {
    if (!grid_) {return false;}
    const double res = grid_->info.resolution;
    const int col = static_cast<int>((x - grid_->info.origin.position.x) / res);
    const int row = static_cast<int>((y - grid_->info.origin.position.y) / res);
    for (int dr = -2; dr <= 2; ++dr) {
      for (int dc = -2; dc <= 2; ++dc) {
        const int r = row + dr, c = col + dc;
        if (r < 0 || c < 0 ||
          r >= static_cast<int>(grid_->info.height) ||
          c >= static_cast<int>(grid_->info.width))
        {
          return false;
        }
        const int8_t value = grid_->data[r * grid_->info.width + c];
        if (value < 0 || value >= 25) {return false;}
      }
    }
    return true;
  }

  bool visible(double x0, double y0, double x1, double y1) const
  {
    const double distance = std::hypot(x1 - x0, y1 - y0);
    const int steps = std::max(2, static_cast<int>(distance / 0.05));
    for (int i = 0; i <= steps; ++i) {
      const double t = static_cast<double>(i) / steps;
      if (!free_at(x0 + t * (x1 - x0), y0 + t * (y1 - y0))) {return false;}
    }
    return true;
  }

  /// Turns the cell path into the few straight legs it really is.
  ///
  /// A path out of the fixed point walks four-connected cells: it staircases
  /// across open floor because a grid has no diagonals, and a robot following
  /// it cell by cell weaves for the same reason. Every cell of it is inside
  /// the winning region, so any straight line that stays inside is safe too,
  /// and the far end of the longest such line is the only point worth steering
  /// at.
  void smooth(const nav_msgs::msg::Path & raw)
  {
    std::vector<Point> points;
    points.reserve(raw.poses.size());
    for (const auto & pose : raw.poses) {
      points.push_back(Point{pose.pose.position.x, pose.pose.position.y});
    }

    route_.clear();
    route_.push_back(points.front());
    std::size_t anchor = 0;
    while (anchor + 1 < points.size()) {
      std::size_t farthest = anchor + 1;
      for (std::size_t i = points.size(); i-- > anchor + 1; ) {
        if (visible(points[anchor].x, points[anchor].y, points[i].x, points[i].y)) {
          farthest = i;
          break;
        }
      }
      route_.push_back(points[farthest]);
      anchor = farthest;
    }
    leg_ = 0;
  }

  /// Once, on the first pose that arrives: does the map frame agree with the
  /// frame the zones are written in?
  ///
  /// If it does, the robot starts within a metre of where warehouse_scenario
  /// says it does. If it does not -- a different spawn, a SLAM configuration
  /// that anchors the map on the robot instead of the world -- then every zone
  /// this node asks for is displaced by exactly that difference, and the run
  /// is worthless. Better to say so on the first message than to let somebody
  /// watch a robot mill around for ten minutes wondering why.
  void check_frames()
  {
    if (frames_checked_) {return;}
    frames_checked_ = true;

    const double dx = x_ - kR1Start.x;
    const double dy = y_ - kR1Start.y;
    const double off = std::sqrt(dx * dx + dy * dy);
    if (off < 1.0) {
      RCLCPP_INFO(get_logger(),
        "map frame agrees with the world: robot at (%.2f, %.2f), "
        "expected (%.2f, %.2f)", x_, y_, kR1Start.x, kR1Start.y);
      return;
    }
    RCLCPP_ERROR(get_logger(),
      "map frame is %.2f m from the world frame: robot reports (%.2f, %.2f) "
      "but warehouse_scenario puts it at (%.2f, %.2f). Every zone this node "
      "asks for is displaced by that much, and most of them are inside a rack. "
      "Expect no route, and do not trust this run.",
      off, x_, y_, kR1Start.x, kR1Start.y);
  }

  void ask(const std::string & zone)
  {
    // The geometry the planner grounds its zones against: one world, because
    // where a dock *is* was never in doubt. What is in doubt lives in the
    // epistemic state, not here.
    std::ostringstream out;
    out << "{\"worlds\": [\"w0\"], \"designated\": [\"w0\"],"
        << " \"relations\": {\"r1\": {\"w0\": [\"w0\"]}},"
        << " \"labels\": {\"w0\": []},"
        << " \"agents\": {\"1\": {\"name\": \"r1\", \"pose\": {\"x\": " << x_
        << ", \"y\": " << y_ << "}}},"
        << " \"zones\": {\"frontier\": " << bounds_json(frontier_box_)
        << ", \"dock_south\": " << bounds_json(zone_named("dock_south"))
        << ", \"dock_north\": " << bounds_json(zone_named("dock_north"))
        << ", \"lane\": " << bounds_json(zone_named("lane"))
        << ", \"corridor\": " << bounds_json(zone_named("corridor"))
        << ", \"bay2\": " << bounds_json(zone_named("bay2"))
        << ", \"bay3\": " << bounds_json(zone_named("bay3"))
        << "}}";

    std_msgs::msg::String snapshot;
    snapshot.data = out.str();
    state_->publish(snapshot);

    epistemic_msgs::msg::MuPathQuery query;
    query.header.frame_id = "map";
    query.header.stamp = now();
    query.agent_id = 1;
    query.goal_zone = zone;
    query.require_epistemic_goal = false;   // where a zone is, is not in doubt
    query_->publish(query);

    asked_zone_ = zone;
    asked_at_ = now();
  }

  // -- exploring ------------------------------------------------------------

  /// How many cells of a zone the map has actually measured as free.
  ///
  /// Worth separating from "no route", because the two look identical from
  /// outside and want opposite reactions. A zone the robot cannot reach is a
  /// routing problem. A zone with no measured cells at all is not a routing
  /// problem: the planner grounds it to nothing, and no amount of asking will
  /// produce a path to a goal that does not exist yet. The second is the
  /// normal state of a warehouse the robot has not walked, and the only cure
  /// is to go and measure it.
  std::size_t measured_cells(const Box & box) const
  {
    if (!grid_) {return 0;}
    const auto & info = grid_->info;
    const int width = static_cast<int>(info.width);
    const int height = static_cast<int>(info.height);
    const double res = info.resolution;

    std::size_t count = 0;
    for (double my = box.min_y; my <= box.max_y; my += res) {
      for (double mx = box.min_x; mx <= box.max_x; mx += res) {
        const int c = static_cast<int>((mx - info.origin.position.x) / res);
        const int r = static_cast<int>((my - info.origin.position.y) / res);
        if (r < 0 || c < 0 || r >= height || c >= width) {continue;}
        const int8_t v = grid_->data[static_cast<std::size_t>(r) * width + c];
        if (v >= 0 && v < 25) {++count;}
      }
    }
    return count;
  }

  /// Has this spot already been stood on and failed to pay?
  ///
  /// Some frontiers cannot be resolved by going to them: the unmeasured cells
  /// they border are behind something, so the robot arrives, the laser reads
  /// the same obstacle again, and the cells stay unmeasured. The frontier is
  /// therefore still a frontier, still the best-scoring one, and the robot
  /// goes back to it forever.
  ///
  /// Once visited, a frontier is spent for the rest of this drive. That is
  /// enough: the map only grows, so anything a second visit would have
  /// revealed, some other frontier now borders too.
  bool spent(double mx, double my) const
  {
    const rclcpp::Time at = now();
    for (const auto & [p, when] : spent_) {
      if ((at - when).seconds() > kSpentFor) {continue;}
      if (std::hypot(p.x - mx, p.y - my) < kSpentRadius) {return true;}
    }
    return false;
  }

  /// How far towards @p toward the robot can actually get, on today's map.
  ///
  /// The fixed point will not route across cells nobody has measured, which is
  /// the whole point of it, and early in a run almost nothing has been
  /// measured -- so the honest answer to "how do I get to the service lane" is
  /// that there is no known way. That is not a reason to stand still. It is a
  /// reason to go as far that way as the map allows, look, and ask again.
  ///
  /// Breadth-first over known-free cells finds everywhere the robot could
  /// drive without leaving measured floor; the one of those closest to where
  /// it is trying to go is where it should be standing when it next asks.
  ///
  /// The clearance test is what makes the answer usable. The planner inflates
  /// obstacles by kInflationCells before it routes, and unknown counts as an
  /// obstacle -- so a cell right on the edge of the measured region, which is
  /// exactly the sort of cell "as far as I can see" would otherwise pick, is
  /// swallowed by that inflation and the planner then reports no route to it.
  /// Requiring the same clearance here asks only for somewhere the planner
  /// will agree the robot can stand.
  /// Somewhere far enough away to be worth the trip, if there is one.
  ///
  /// Nav2 reaches a goal within a quarter of a metre and reports success, so a
  /// target 0.6 m off is a hop: the robot shuffles, arrives, picks another,
  /// and the map grows by almost nothing each time. Ask for a metre and a half
  /// first, and settle for less only when the known floor really has nothing
  /// further out.
  bool frontier_toward(const Box & toward, Point & out) const
  {
    return frontier_toward(toward, out, 1.5) ||
           frontier_toward(toward, out, 0.6);
  }

  bool frontier_toward(const Box & toward, Point & out, double at_least) const
  {
    if (!grid_) {return false;}

    const auto & info = grid_->info;
    const int width = static_cast<int>(info.width);
    const int height = static_cast<int>(info.height);
    if (width <= 0 || height <= 0) {return false;}

    const double res = info.resolution;
    const double ox = info.origin.position.x;
    const double oy = info.origin.position.y;

    const int start_c = static_cast<int>((x_ - ox) / res);
    const int start_r = static_cast<int>((y_ - oy) / res);
    if (start_c < 0 || start_r < 0 || start_c >= width || start_r >= height) {
      return false;
    }

    const double goal_x = (toward.min_x + toward.max_x) / 2.0;
    const double goal_y = (toward.min_y + toward.max_y) / 2.0;

    const auto known_free = [&](int r, int c) {
        const int8_t v = grid_->data[static_cast<std::size_t>(r) * width + c];
        return v >= 0 && v < 25;
      };

    // Room for the robot and for the planner's own inflation, on the same
    // grid the planner will use. Without this the target is a cell the
    // planner will not route to and the robot stops short of a place it can
    // see.
    // Room for the robot: no *occupied* cell within the inflation radius.
    //
    // Occupied, and not merely "not known free". This has to be the same test
    // the planner applies or the target is one it will not route to, and the
    // planner grows obstacles from cells it has actually measured as occupied
    // -- see inflate_obstacles() in mu_path_planner_node, which skips every
    // cell whose state is not Occupied.
    //
    // Demanding a full disc of known-free instead is the stricter-looking
    // mistake, and it rejects precisely the cells worth going to: a frontier
    // has unmeasured cells beside it by definition, so a disc centred on one
    // is never all known-free. Under that test the only acceptable targets are
    // deep inside ground already mapped, the robot explores nowhere, and a
    // zone a metre past the edge of the map stays unreachable forever. Off the
    // grid entirely is unmeasured too, and no more of an obstacle than
    // unmeasured cells inside it.
    const auto clear_within = [&](int r, int c, int radius) {
        for (int dr = -radius; dr <= radius; ++dr) {
          for (int dc = -radius; dc <= radius; ++dc) {
            if (dr * dr + dc * dc > radius * radius) {continue;}
            const int nr = r + dr, nc = c + dc;
            if (nr < 0 || nc < 0 || nr >= height || nc >= width) {continue;}
            if (grid_->data[static_cast<std::size_t>(nr) * width + nc] > 65) {
              return false;
            }
          }
        }
        return true;
      };
    const auto has_clearance = [&](int r, int c) {
        return clear_within(r, c, kInflationCells);
      };

    // And an unmeasured cell close enough that standing here would measure
    // it. This is what makes the target a frontier rather than merely the
    // reachable cell nearest the goal.
    //
    // Without it the robot walks to whichever corner of the measured region
    // points at the goal and stops there: the map stops growing, the goal
    // stays unreachable, and it shuffles on the spot. The map has to get
    // bigger for the run to progress, so the target has to be somewhere that
    // makes it bigger.
    constexpr int kLookFrom = kInflationCells + 5;
    const auto sees_unknown = [&](int r, int c) {
        for (int dr = -kLookFrom; dr <= kLookFrom; ++dr) {
          for (int dc = -kLookFrom; dc <= kLookFrom; ++dc) {
            const int nr = r + dr, nc = c + dc;
            // Beyond the published grid is unmeasured too, and standing here
            // would measure it.
            if (nr < 0 || nc < 0 || nr >= height || nc >= width) {return true;}
            if (grid_->data[static_cast<std::size_t>(nr) * width + nc] < 0) {
              return true;
            }
          }
        }
        return false;
      };

    // Breadth-first, carrying how far each cell is from the robot over
    // measured floor -- not as the crow flies. That number is what lets the
    // score below tell "a step away, pointing at the goal" from "right across
    // the building, pointing at the goal".
    std::vector<bool> seen(static_cast<std::size_t>(width) * height, false);
    std::deque<std::tuple<int, int, int>> queue;
    seen[static_cast<std::size_t>(start_r) * width + start_c] = true;
    queue.emplace_back(start_r, start_c, 0);

    bool found = false;
    double best = std::numeric_limits<double>::infinity();
    const int dr4[] = {-1, 1, 0, 0};
    const int dc4[] = {0, 0, -1, 1};

    while (!queue.empty()) {
      const auto [r, c, depth] = queue.front();
      queue.pop_front();

      const double mx = ox + (c + 0.5) * res;
      const double my = oy + (r + 0.5) * res;

      // Somewhere it is already standing is no use: driving there moves
      // nothing and the next question is the same question.
      // See frontier_toward above for why the bar is a metre and a half.
      if (std::hypot(mx - x_, my - y_) > at_least && has_clearance(r, c) &&
        !spent(mx, my) && sees_unknown(r, c))
      {
        // Nearest frontier first, with the goal only breaking ties.
        //
        // Scoring mostly on distance-to-the-goal is the obvious thing and it
        // oscillates: two frontiers on opposite sides of the robot both point
        // roughly at the destination, the robot drives to one, that changes
        // which of them looks better, and it drives back. It spent whole runs
        // crossing and recrossing the same six metres.
        //
        // Going to the nearest unexplored edge instead is the oldest
        // frontier-exploration rule there is, and it does not oscillate: the
        // robot sweeps outward, each frontier it reaches is spent, and the
        // next nearest is the one beside it. The map grows steadily in every
        // direction rather than lurching towards wherever the goal happens to
        // be, and a warehouse mapped steadily is a warehouse the goal ends up
        // inside.
        const double reach = depth * res;

        // Open floor is worth going out of the way for.
        //
        // The building leaves gaps that clear the robot on paper and trap it
        // in practice -- 0.43 m between the pallet jack and the south wall.
        // A target in one is legal, because the planner will route there while
        // the far side of the gap is still unmeasured, and useless, because
        // the robot wedges in the mouth of it and the frontier expires
        // unreached. Preferring somewhere with a robot's width to spare sends
        // it round the jack instead of into the gap beside it, which is the
        // way a person would go.
        const bool roomy = clear_within(r, c, 2 * kInflationCells);
        const double score = reach +
          0.5 * std::hypot(mx - goal_x, my - goal_y) - (roomy ? 1.5 : 0.0);
        if (score < best) {
          best = score;
          out = Point{mx, my};
          found = true;
        }
      }

      for (int d = 0; d < 4; ++d) {
        const int nr = r + dr4[d], nc = c + dc4[d];
        if (nr < 0 || nc < 0 || nr >= height || nc >= width) {continue;}
        const auto index = static_cast<std::size_t>(nr) * width + nc;
        if (seen[index] || !known_free(nr, nc)) {continue;}
        seen[index] = true;
        queue.emplace_back(nr, nc, depth + 1);
      }
    }
    return found;
  }

  /// Is the robot in fact in the zone? Reaching the end of the route is not
  /// the same question: early on, the map reaches only a metre or two ahead,
  /// so the route to a zone off the edge of it ends a metre away and running
  /// out of route would otherwise be reported as arrival. An action that says
  /// it put the robot somewhere it did not is worse than one that takes
  /// longer, because everything downstream believes it.
  bool inside(const std::string & zone) const
  {
    const Box box = zone_named(zone);
    constexpr double kMargin = 0.25;
    return x_ > box.min_x - kMargin && x_ < box.max_x + kMargin &&
           y_ > box.min_y - kMargin && y_ < box.max_y + kMargin;
  }

  // -- handing it to Nav2 -----------------------------------------------------

  /// Send @p target to Nav2, unless it is already going there.
  ///
  /// The pose comes from the mu-calculus, not from Nav2: it is the end of the
  /// route the fixed point returned, which is a place it has certified as
  /// reachable over floor that has actually been measured. Nav2 is told where
  /// to go and left to work out how -- its costmap, its local planner, and
  /// when it wedges itself, its recovery behaviours.
  ///
  /// That is the division worth keeping. Nav2 asked for a zone directly would
  /// happily set off across unmeasured floor, which is what a navigation stack
  /// is for and exactly what this repository is arguing against.
  void steer_to(const Point & target)
  {
    if (!nav_->action_server_is_ready()) {
      return;
    }
    if (goal_live_ &&
      std::hypot(target.x - sent_.x, target.y - sent_.y) < kGoalMoved)
    {
      return;   // already on its way there
    }
    // A rejected goal clears goal_live_, and without this the next cycle
    // sends the same one again -- five times a second, forever, while Nav2 is
    // still coming up.
    if ((now() - sent_at_).seconds() < 2.0) {
      return;
    }

    NavigateToPose::Goal goal;
    goal.pose.header.frame_id = "map";
    goal.pose.header.stamp = now();
    goal.pose.pose.position.x = target.x;
    goal.pose.pose.position.y = target.y;
    // Face along the way it is going, so the first thing Nav2 does is not a
    // pirouette. The yaw tolerance is wide open, so this is a hint only.
    const double heading = std::atan2(target.y - y_, target.x - x_);
    goal.pose.pose.orientation.z = std::sin(heading / 2.0);
    goal.pose.pose.orientation.w = std::cos(heading / 2.0);

    rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
    options.goal_response_callback =
      [this](const GoalHandle::SharedPtr & handle) {
        goal_live_ = handle != nullptr;
      };
    options.result_callback =
      [this](const GoalHandle::WrappedResult & result) {
        goal_live_ = false;
        if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
          // Not a failure of the mission: Nav2 gave up on this pose, and the
          // next cycle asks the fixed point for somewhere else to stand.
          RCLCPP_INFO(get_logger(), "nav2 did not reach (%.2f, %.2f)",
            sent_.x, sent_.y);
        }
      };

    nav_->async_send_goal(goal, options);
    sent_ = target;
    sent_at_ = now();
    goal_live_ = true;
    RCLCPP_INFO(get_logger(), "nav2: go to (%.2f, %.2f)", target.x, target.y);
  }

  void stop_navigating()
  {
    if (goal_live_) {
      nav_->async_cancel_all_goals();
      goal_live_ = false;
    }
  }

  // -- the action -----------------------------------------------------------

  void on_activate_action()
  {
    route_.clear();
    leg_ = 0;
    stop_navigating();
    empty_answer_ = false;
    have_frontier_ = false;
    spent_.clear();
    started_ = now();
    stall_since_ = now();
    stall_x_ = x_;
    stall_y_ = y_;
    asked_at_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  }

  void do_work() override
  {
    const auto & args = get_arguments();
    if (args.size() < 3) {
      finish(false, 0.0, "goto_zone takes a robot, a source and a destination");
      return;
    }
    const std::string destination = args[2];

    if (!pose()) {
      send_feedback(0.0, "waiting for map -> base_footprint");
      return;
    }
    if (!grid_) {
      send_feedback(0.0, "waiting for a map");
      return;
    }

    if (asked_at_.nanoseconds() == 0) {on_activate_action();}

    const double elapsed = (now() - started_).seconds();
    if (elapsed > get_parameter("timeout").as_double()) {
      stop_navigating();
      asked_at_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
      finish(false, 0.5, "gave up driving to " + destination);
      return;
    }

    // Nothing here backs the robot out of anything or waits for it to stop
    // being stuck. Nav2 owns that, and it owns it properly -- clear the
    // costmap, reverse, spin, replan. Two recovery behaviours running at once
    // is worse than either alone.
    //
    // A route is asked for again on a slow cadence while the robot drives:
    // the map grows as it goes, and a zone that was out of reach a moment ago
    // -- or reachable only the long way round -- may not be now.
    if ((now() - asked_at_).seconds() > kRequery || route_.empty()) {
      // While the destination itself is unreachable, the frontier is asked
      // for instead. That is not a detour around the answer: the fixed point
      // is refusing to cross cells nobody has observed, and the way to change
      // its mind is to go and observe them. So the robot drives to the edge of
      // what it knows, in the direction it wants to go, and asks again.
      // Committed to a place to look, if there is one.
      //
      // The destination is only re-asked once the robot has got where it was
      // going. Asking it every few seconds instead looks harmless and is not:
      // the answer to "route to the frontier?" is yes, which clears the
      // no-route flag, which makes the next cycle ask for the destination
      // again, fail again, and choose a *different* frontier -- so the robot
      // alternates between two of them and travels nowhere.
      std::string zone = destination;
      if (have_frontier_) {
        const bool arrived =
          std::hypot(frontier_.x - x_, frontier_.y - y_) < 0.5;
        const bool expired = (now() - frontier_at_).seconds() > kFrontierHold;
        if (arrived || expired) {
          spent_.emplace_back(frontier_, now());
          have_frontier_ = false;   // look again from here
        } else {
          zone = "frontier";
        }
      }

      if (!have_frontier_ && empty_answer_) {
        Point target{0.0, 0.0};
        if (frontier_toward(zone_named(destination), target)) {
          frontier_ = target;
          have_frontier_ = true;
          frontier_at_ = now();
          frontier_box_ = Box{target.x - 0.20, target.y - 0.20,
            target.x + 0.20, target.y + 0.20};
          zone = "frontier";
          RCLCPP_INFO(get_logger(),
            "no route to %s yet; going to look from (%.2f, %.2f)",
            destination.c_str(), target.x, target.y);
        }
      }
      ask(zone);
    }

    // A line every few seconds, so a run that goes wrong afterwards can be
    // read rather than guessed at.
    if ((now() - reported_at_).seconds() > 5.0) {
      reported_at_ = now();
      const std::size_t measured = measured_cells(zone_named(destination));

      std_msgs::msg::String saying;
      if (have_frontier_) {
        saying.data = "driving to " + destination + ": no measured route yet, "
          "so going to look from (" + trimmed(frontier_.x) + ", " +
          trimmed(frontier_.y) + ")";
      } else if (empty_answer_ && measured == 0) {
        saying.data = "driving to " + destination +
          ": that floor has never been measured, so there is nothing to route to";
      } else if (empty_answer_) {
        saying.data = "driving to " + destination +
          ": measured, but no route through measured floor yet";
      } else {
        saying.data = "driving to " + destination + ": following a route of " +
          std::to_string(route_.size()) + " legs";
      }
      saying.data = get_parameter("robot").as_string() + ": " + saying.data;
      activity_->publish(saying);

      RCLCPP_INFO(get_logger(),
        "to %s: at (%.2f, %.2f), asked '%s', %zu legs%s; %zu cells of %s "
        "measured so far",
        destination.c_str(), x_, y_, asked_zone_.c_str(), route_.size(),
        empty_answer_ ?
        (measured == 0 ? ", and the zone is not on the map yet" :
        ", last answer was no route") : "",
        measured, destination.c_str());
    }

    if (inside(destination)) {
      stop_navigating();
      asked_at_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
      finish(true, 1.0, "at " + destination);
      return;
    }

    // Drive to the zone. Nav2 has the floor plan and can plan over the whole
    // building, so this is a destination rather than a direction.
    //
    // The mu-calculus answer is still asked for and still reported -- it is
    // what the caption says and what the epistemic layer above reasons with --
    // but it no longer decides whether the wheels turn. Those are two
    // different questions and conflating them is what had the robot exploring
    // a warehouse it had been given a map of.
    // The nearest part of the zone, not the middle of it.
    //
    // The middle of a bay is where the pallet stands, and a goal pose inside
    // an obstacle is one Nav2 will never report reaching: the robot stops at
    // the mouth and the action times out. Clamping the robot's own position
    // into the box gives the closest place it can stand, which is the mouth
    // end -- near enough for `inside()`, and not on top of the thing it came
    // to look at.
    const Box box = zone_named(destination);
    constexpr double kEdge = 0.3;
    steer_to(Point{
        std::clamp(x_, box.min_x + kEdge, box.max_x - kEdge),
        std::clamp(y_, box.min_y + kEdge, box.max_y - kEdge)});

    send_feedback(std::min(0.95, elapsed / 60.0), "driving to " + destination);
  }

  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr activity_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_;
  Point sent_{0.0, 0.0};
  rclcpp::Time sent_at_{0, 0, RCL_ROS_TIME};
  bool goal_live_{false};
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_;
  rclcpp::Publisher<epistemic_msgs::msg::MuPathQuery>::SharedPtr query_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_;
  std::shared_ptr<tf2_ros::Buffer> buffer_;
  std::shared_ptr<tf2_ros::TransformListener> listener_;

  nav_msgs::msg::OccupancyGrid::SharedPtr grid_;
  /// Where the robot is currently going to look, when the goal is out of
  /// reach. Republished with every snapshot so the planner can ground it.
  Box frontier_box_{-0.2, -0.2, 0.2, 0.2};
  Point frontier_{0.0, 0.0};
  std::vector<std::pair<Point, rclcpp::Time>> spent_;
  bool have_frontier_{false};
  rclcpp::Time frontier_at_{0, 0, RCL_ROS_TIME};
  std::vector<Point> route_;
  std::size_t leg_{0};
  bool empty_answer_{false};
  std::string asked_zone_;
  rclcpp::Time asked_at_{0, 0, RCL_ROS_TIME};
  rclcpp::Time started_{0, 0, RCL_ROS_TIME};
  rclcpp::Time stall_since_{0, 0, RCL_ROS_TIME};
  bool frames_checked_{false};
  double stall_x_{0.0}, stall_y_{0.0};
  rclcpp::Time recovery_until_{0, 0, RCL_ROS_TIME};
  rclcpp::Time reported_at_{0, 0, RCL_ROS_TIME};
  double x_{0.0}, y_{0.0}, yaw_{0.0};
  double ahead_{std::numeric_limits<double>::infinity()};
  double avoid_{0.0};
  double clearer_{1.0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DriveAction>();
  node->set_parameter(rclcpp::Parameter("action_name", "goto_zone"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
