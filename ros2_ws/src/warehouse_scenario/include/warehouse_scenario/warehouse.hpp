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
// The floor plan follows the RoboticsAcademy multi-robot Amazon warehouse
// exercise: shelf blocks with aisles between them, a cross corridor at each
// end, and loading docks on the east wall.  What it adds is the part that
// exercise leaves out — what the robots do not know — and it adds it to the
// map rather than around it:
//
//   * a corridor nobody has looked into is `unknown`, not free, so the least
//     fixed point halts at its mouth instead of driving through it;
//   * which aisle holds the pallet is a disagreement between two worlds one
//     robot cannot tell apart and another can, so the goal `the robot knows
//     it arrived` needs a sensing action and the goal `the robot is there`
//     does not.
//
// One definition of the layout serves the map writer, the offline run, the
// ROS driver and the tests.  Nothing here talks to ROS.
// ---------------------------------------------------------------------------

namespace warehouse_scenario
{

using mu_path_planner::CellIdx;
using mu_path_planner::CellState;

// -- the warehouse, in metres ----------------------------------------------

inline constexpr double kResolution = 0.10;
inline constexpr double kWidthM = 24.0;
inline constexpr double kHeightM = 14.0;

inline constexpr uint32_t kWidth = 240;    // kWidthM / kResolution
inline constexpr uint32_t kHeight = 140;   // kHeightM / kResolution

/// A metric rectangle, in the map frame.
struct Box
{
  double min_x, min_y, max_x, max_y;
};

/// Where the two robots stand at the start of a run.
struct Point
{
  double x, y;
};

inline constexpr Box kDock1{21.0, 2.0, 23.5, 4.5};
inline constexpr Box kDock2{21.0, 9.5, 23.5, 12.0};

/// The two bays a pallet could be in: the mouth of aisle 2 and of aisle 3.
inline constexpr Box kBayAisle2{6.0, 3.4, 7.6, 4.6};
inline constexpr Box kBayAisle3{10.0, 3.4, 11.6, 4.6};

/// The only way from the aisles to the east wall, and the thing stage_a has
/// never observed.
inline constexpr double kEastCorridorX0 = 19.0;
inline constexpr double kEastCorridorX1 = 20.6;

inline constexpr Point kR1Start{2.0, 1.6};
inline constexpr Point kR2Start{2.0, 12.4};

// -- the floor --------------------------------------------------------------

/// Occupancy values, in the convention of nav_msgs/OccupancyGrid.
inline constexpr int8_t kFree = 0;
inline constexpr int8_t kOccupied = 100;
inline constexpr int8_t kUnknown = -1;

/// The warehouse floor as an occupancy grid, row 0 at the south edge.
/// @param east_corridor_observed  false leaves the east corridor unobserved,
///        which is the whole of the difference between the two stages.
std::vector<int8_t> build_grid(bool east_corridor_observed);

/// Flat index of the cell containing a point given in map metres.
CellIdx cell_of(double x, double y);

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

/// Nothing in dispute: one world, both docks grounded on the map.
std::string snapshot_docks();

/// The pallet is in aisle 2 or in aisle 3, and r1 cannot tell which.
std::string snapshot_pallet();

/// The same disagreement, with r2 added: r2 walked past the aisles and tells
/// the worlds apart, so what r1 has to go and find out, r2 already knows.
std::string snapshot_fleet();

// -- the cases --------------------------------------------------------------

/// One question put to the planner, and what the scenario expects back.
struct Case
{
  std::string name;
  bool east_corridor_observed;   ///< which of the two maps this case runs on
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

/// Cell index to a point in map metres, at the centre of the cell.
Point point_of(CellIdx cell);

}  // namespace warehouse_scenario
