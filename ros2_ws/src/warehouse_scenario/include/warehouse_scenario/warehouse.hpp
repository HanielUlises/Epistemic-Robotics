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

#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "mu_path_planner/epistemic_state.hpp"
#include "mu_path_planner/mu_calculus.hpp"

// ---------------------------------------------------------------------------
// The warehouse scenario.
//
// The floor is the one the RoboticsAcademy multi-robot Amazon warehouse
// exercise runs on: AWS RoboMaker's small warehouse world, rasterised from the
// collision meshes Gazebo itself uses for that world by
// scenarios/warehouse/tools/rasterize_world.py.  Nothing here is drawn to
// resemble the exercise; the building, the six rack rows, the shelf along the
// west wall and the clutter between them are that world's own geometry, at
// that world's own coordinates.  A rack the fixed point drives around is the
// rack the laser hits.
//
// What the exercise asks for is a central task planner that assigns the jobs.
// What it never asks is how a robot comes to *know* where the pallet is, or
// what it should do about a part of the floor it has never seen.  Both are the
// subject here, and both are answered on the map rather than around it:
//
//   * a stage is not painted but ray cast.  What a robot knows about the floor
//     is what a laser standing where it has stood could actually have
//     returned, so an aisle between two racks -- which a robot can only enter
//     and only see down from its mouth on the service lane -- stays unknown
//     until somebody drives that lane.  Unknown is not free, so the least
//     fixed point halts where the measurements stop rather than driving on
//     through floor nobody has read;
//
//   * which aisle holds the pallet is a disagreement between two worlds one
//     robot cannot tell apart and another can, so the goal `the robot knows it
//     arrived` needs a sensing action and the goal `the robot is there` does
//     not.
//
// One definition of the layout serves the map writer, the offline run, the ROS
// driver and the tests.  Nothing here talks to ROS.
// ---------------------------------------------------------------------------

namespace warehouse_scenario
{

using mu_path_planner::CellIdx;
using mu_path_planner::CellState;

// -- the grid ---------------------------------------------------------------
//
// The map frame is the world frame of the AWS world, so a pose read off Gazebo
// is a pose the snapshot can quote.  These four numbers are also written into
// maps/aws_small_warehouse.yaml and into the generator; changing one means
// changing all of them.

inline constexpr double kResolution = 0.05;
inline constexpr uint32_t kWidth = 281;    // 14.05 m
inline constexpr uint32_t kHeight = 421;   // 21.05 m
inline constexpr double kOriginX = -7.00;
inline constexpr double kOriginY = -10.50;

/// The floor plan, alternating run lengths starting with free, row 0 south.
/// Defined in the generated src/floorplan.gen.cpp.
extern const uint32_t kFloorplanRuns[];
extern const std::size_t kFloorplanRunCount;

/// A metric rectangle, in the map frame.
struct Box
{
  double min_x, min_y, max_x, max_y;
};

/// A point in the map frame.
struct Point
{
  double x, y;
};

// -- what the world put where ------------------------------------------------
//
// Read off the AWS world's own model poses and mesh extents.  The six rack
// rows all span x in [2.77, 6.69]; the gaps between them are the aisles.  The
// racks stop 0.15 m short of the east wall, which is a gap on the floor plan
// and none at all to a robot 0.21 m across -- see kRobotRadiusM -- so an
// aisle has the one mouth, on the service lane.

inline constexpr double kRackMinX = 2.77;
inline constexpr double kRackMaxX = 6.69;

/// The lane along the rack fronts: the only floor every aisle opens onto.
inline constexpr double kServiceLaneX = 2.20;

/// The bay at the mouth end of each of the two aisles a pallet could be in.
/// Aisle 2 is the gap between the racks at y = -1.68 and y = -2.60, aisle 3
/// the gap between those at y = -3.48 and y = -4.39.  Each bay is drawn
/// inside what is left of its aisle once the racks are grown by the robot's
/// radius, so a bay is somewhere the robot can actually stand.
inline constexpr Box kBayAisle2{3.6, -2.45, 5.0, -1.83};
inline constexpr Box kBayAisle3{3.6, -4.24, 5.0, -3.63};

/// The bay in aisle 4, which is where the deep end of the rack block is put
/// to work.
inline constexpr Box kBayAisle4{3.6, -6.15, 5.0, -5.43};

/// The two ends of the floor the pallets move between: shipping by the pallet
/// jack in the south, receiving under the north wall.
inline constexpr Box kDockSouth{-4.4, -9.9, -2.8, -8.5};
inline constexpr Box kDockNorth{-4.4, 5.5, -2.8, 7.0};

inline constexpr Point kR1Start{-3.5, -9.3};
inline constexpr Point kR2Start{-3.5, 6.2};

/// The range of the TurtleBot3's LDS, which is what makes a stage a stage.
inline constexpr double kLaserRangeM = 3.5;

/// The radius of the TurtleBot3 the demo drives, and the amount every
/// obstacle is grown by before the fixed point sees the floor.
///
/// This is not decoration.  The racks stop 0.15 m short of the east wall, and
/// to a planner that treats a robot as a point that gap is a corridor joining
/// every aisle behind the racks -- so the fixed point would return routes down
/// it, and no robot could drive one.  Growing the obstacles by the radius is
/// what makes the grid the robot's configuration space rather than the
/// building's floor, and it closes that gap while leaving 0.7 m of each aisle.
inline constexpr double kRobotRadiusM = 0.105;

// -- the floor --------------------------------------------------------------

/// Occupancy values, in the convention of nav_msgs/OccupancyGrid.
inline constexpr int8_t kFree = 0;
inline constexpr int8_t kOccupied = 100;
inline constexpr int8_t kUnknown = -1;

/// How much of the warehouse has been looked at.
///
/// Neither stage is a mask drawn over the floor: each is the set of cells a
/// laser standing at that stage's vantage points could have returned, ray cast
/// against the building itself.  What separates them is only where the fleet
/// has stood.
enum class Stage
{
  /// Each robot has looked around the end of the building it started at and
  /// no further: r1 the shipping floor in the south, r2 receiving in the
  /// north.  The floor between them has never been measured, and neither has
  /// anything east of it.
  FirstPass,

  /// The same, and then the sweep: up the west corridor, which joins the two
  /// ends into one known floor, and up the service lane, which is what puts a
  /// laser at the mouth of each aisle.
  AfterTheSweep,
};

/// Where the fleet had stood by the end of a stage.
std::vector<Point> vantages(Stage stage);

/// The floor plan alone: the AWS world's obstacles at their true extent,
/// nothing grown and nothing unobserved.  This is the building, and it is what
/// a laser measures.
std::vector<int8_t> build_floorplan();

/// The same floor plan with every obstacle grown by @p radius_m: the floor as
/// the robot's centre may move over it.  This is what the fixed point plans
/// on, and the difference between the two is a robot's width.
std::vector<int8_t> inflate(const std::vector<int8_t> & floorplan, double radius_m);

/// The warehouse as the fleet has it at @p stage: the floor plan grown by the
/// robot's radius, with every cell no laser reached left unobserved rather
/// than free.  The rays are cast against the building, not against the grown
/// obstacles, because a laser measures the wall where the wall is.
std::vector<int8_t> build_grid(Stage stage);

/// The cells a laser at @p from could return, out to @p range_m, stopping at
/// the first obstacle along each ray.  This is the whole of the difference
/// between the stages, and it is exported so a test can ask it directly.
std::vector<bool> observed_from(
  const std::vector<int8_t> & floorplan, const std::vector<Point> & from,
  double range_m);

/// Flat index of the cell containing a point given in map metres.
CellIdx cell_of(double x, double y);

/// Cell index to a point in map metres, at the centre of the cell.
Point point_of(CellIdx cell);

/// The reading of each cell once the thresholds are applied: the same
/// three-valued abstraction the planner node performs on a live map.
std::vector<CellState> classify(const std::vector<int8_t> & grid);

/// The 4-connected graph the fixed point runs over. A cell that is not free —
/// occupied or merely unobserved — is an obstacle to it.
mu_path_planner::OccupancyGraph build_graph(const std::vector<CellState> & state);

/// The grid geometry the snapshot parser needs to resolve poses and metric
/// zone bounds into cells.
mu_path_planner::GridInfo grid_info();

// -- what the robots know ---------------------------------------------------

/// Nothing in dispute: one world, the docks and aisle 4 grounded on the map.
std::string snapshot_docks();

/// The pallet is in aisle 2 or in aisle 3, and r1 cannot tell which.
std::string snapshot_pallet();

/// The same disagreement, with r2 added: r2 walked the lane past both aisle
/// mouths and tells the worlds apart, so what r1 has to go and find out, r2
/// already knows.
std::string snapshot_fleet();

// -- the cases --------------------------------------------------------------

/// One question put to the planner, and what the scenario expects back.
struct Case
{
  std::string name;
  Stage stage;                   ///< which of the two maps this case runs on
  std::string snapshot;          ///< "docks", "pallet" or "fleet"
  uint32_t agent_id;
  std::string goal_zone;
  bool require_epistemic_goal;
  std::string safety_formula_json;   ///< empty means ¬obstacle
  std::string expect;                ///< what it is here to show, in words
};

std::vector<Case> cases();

/// The snapshot text a case names.
std::string snapshot_for(const Case & c);

/// How far a sensor reads, in cells: 1.5 m over the grid, which is enough to
/// settle a bay from inside the aisle it sits in and not from the lane.
inline constexpr uint32_t kSensorRangeCells = 30;

// -- running one --------------------------------------------------------------

/// What one case produced, in cells and in metres.
struct Outcome
{
  bool ok{false};
  std::string error;

  CellIdx start{0};
  std::size_t goal_cells{0};
  std::size_t known_goal_cells{0};
  std::size_t disputed_cells{0};
  std::size_t sensing_cells{0};
  std::size_t safe_cells{0};

  std::size_t winning_region{0};
  uint32_t iterations{0};

  std::vector<CellIdx> path;
  std::vector<CellIdx> sensing_waypoints;
};

/// Resolves the case against the map and the snapshot and takes the fixed
/// point, with no ROS in the way. The planner node does exactly this once a
/// map, a state and a query have arrived; running it here is what makes the
/// scenario reproducible without a graph.
Outcome run(const Case & c);

}  // namespace warehouse_scenario
