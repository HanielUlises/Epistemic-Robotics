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

#include "warehouse_scenario/warehouse.hpp"

#include <cmath>
#include <sstream>
#include <stdexcept>

namespace warehouse_scenario
{
namespace
{

using mu_path_planner::OccupancyGraph;

/// Fills the half-open metric rectangle [x0, x1) x [y0, y1).
///
/// The bounds are rounded rather than truncated: every dimension in this file
/// is a multiple of the resolution, and 20.6 / 0.1 is 205.999... in binary,
/// which truncates one column short and leaves the corridor overlapping the
/// wall beside it.
void fill(std::vector<int8_t> & grid, double x0, double y0, double x1, double y1,
  int8_t value)
{
  const auto c0 = static_cast<uint32_t>(std::lround(x0 / kResolution));
  const auto c1 = static_cast<uint32_t>(std::lround(x1 / kResolution));
  const auto r0 = static_cast<uint32_t>(std::lround(y0 / kResolution));
  const auto r1 = static_cast<uint32_t>(std::lround(y1 / kResolution));

  for (uint32_t row = r0; row < r1 && row < kHeight; ++row) {
    for (uint32_t col = c0; col < c1 && col < kWidth; ++col) {
      grid[static_cast<std::size_t>(row) * kWidth + col] = value;
    }
  }
}

std::string bounds_json(const Box & box)
{
  std::ostringstream out;
  out << "{\"bounds\": {\"min_x\": " << box.min_x << ", \"min_y\": " << box.min_y
      << ", \"max_x\": " << box.max_x << ", \"max_y\": " << box.max_y << "}}";
  return out.str();
}

std::string agent_json(uint32_t id, const char * name, const Point & at)
{
  std::ostringstream out;
  out << "\"" << id << "\": {\"name\": \"" << name
      << "\", \"pose\": {\"x\": " << at.x << ", \"y\": " << at.y << "}}";
  return out.str();
}

/// The perimeter, the racks and the wall that makes the east corridor the
/// only way through. Shared by both stages, since a stage differs from the
/// other only in what has been looked at.
void build_structure(std::vector<int8_t> & grid)
{
  constexpr double kWall = 0.3;
  fill(grid, 0.0, 0.0, kWidthM, kWall, kOccupied);
  fill(grid, 0.0, kHeightM - kWall, kWidthM, kHeightM, kOccupied);
  fill(grid, 0.0, 0.0, kWall, kHeightM, kOccupied);
  fill(grid, kWidthM - kWall, 0.0, kWidthM, kHeightM, kOccupied);

  constexpr double kShelfWidth = 1.6;
  constexpr double kShelfY0 = 3.0;
  constexpr double kShelfY1 = 11.0;
  for (const double x : {4.0, 8.0, 12.0, 16.0}) {
    fill(grid, x, kShelfY0, x + kShelfWidth, kShelfY1, kOccupied);
  }

  fill(grid, kEastCorridorX1, kShelfY0, kEastCorridorX1 + 0.4, kShelfY1, kOccupied);
}

}  // namespace

// ---------------------------------------------------------------------------

std::vector<int8_t> build_grid(bool east_corridor_observed)
{
  std::vector<int8_t> grid(static_cast<std::size_t>(kWidth) * kHeight, kFree);
  build_structure(grid);

  if (!east_corridor_observed) {
    // A corridor nobody has looked into. Not free, not occupied: unknown.
    fill(grid, kEastCorridorX0, 0.3, kEastCorridorX1, kHeightM - 0.3, kUnknown);
  }
  return grid;
}

CellIdx cell_of(double x, double y)
{
  const auto col = static_cast<int64_t>(x / kResolution);
  const auto row = static_cast<int64_t>(y / kResolution);
  if (col < 0 || row < 0 || col >= kWidth || row >= kHeight) {
    throw std::out_of_range("point is off the map");
  }
  return static_cast<CellIdx>(row) * kWidth + static_cast<CellIdx>(col);
}

Point point_of(CellIdx cell)
{
  const uint32_t col = cell % kWidth;
  const uint32_t row = cell / kWidth;
  return Point{(col + 0.5) * kResolution, (row + 0.5) * kResolution};
}

std::vector<CellState> classify(const std::vector<int8_t> & grid)
{
  // The thresholds the planner node defaults to, which are nav2's. The band
  // between them is a cell that was seen without being settled, and it is
  // not free: neither it nor an unobserved cell may be driven through.
  constexpr int kFreeBelow = 25;
  constexpr int kOccupiedAbove = 65;

  std::vector<CellState> state(grid.size(), CellState::Unknown);
  for (std::size_t i = 0; i < grid.size(); ++i) {
    const int value = grid[i];
    if (value >= 0 && value < kFreeBelow) {
      state[i] = CellState::Free;
    } else if (value > kOccupiedAbove) {
      state[i] = CellState::Occupied;
    }
  }
  return state;
}

OccupancyGraph build_graph(const std::vector<CellState> & state)
{
  OccupancyGraph graph;
  graph.width = kWidth;
  graph.height = kHeight;
  graph.obstacle.assign(state.size(), true);
  for (std::size_t i = 0; i < state.size(); ++i) {
    graph.obstacle[i] = state[i] != CellState::Free;
  }
  graph.build_adjacency();
  return graph;
}

mu_path_planner::GridInfo grid_info()
{
  mu_path_planner::GridInfo info;
  info.width = kWidth;
  info.height = kHeight;
  info.resolution = kResolution;
  info.origin_x = 0.0;
  info.origin_y = 0.0;
  return info;
}

// ---------------------------------------------------------------------------

std::string snapshot_docks()
{
  std::ostringstream out;
  out << "{\n"
      << "  \"worlds\": [\"w0\"],\n"
      << "  \"designated\": [\"w0\"],\n"
      << "  \"relations\": {\"r1\": {\"w0\": [\"w0\"]}, \"r2\": {\"w0\": [\"w0\"]}},\n"
      << "  \"labels\": {\"w0\": [\"link_up\"]},\n"
      << "  \"agents\": {" << agent_json(1, "r1", kR1Start) << ", "
      << agent_json(2, "r2", kR2Start) << "},\n"
      << "  \"zones\": {\"dock_1\": " << bounds_json(kDock1)
      << ", \"dock_2\": " << bounds_json(kDock2) << "}\n"
      << "}\n";
  return out.str();
}

std::string snapshot_pallet()
{
  // r1 cannot tell w0 from w1, and the two disagree about which bay is the
  // pallet's. Every cell they disagree about is a cell sensing is for.
  std::ostringstream out;
  out << "{\n"
      << "  \"worlds\": [\"w0\", \"w1\"],\n"
      << "  \"designated\": [\"w0\"],\n"
      << "  \"relations\": {\"r1\": {\"w0\": [\"w0\", \"w1\"], "
      << "\"w1\": [\"w0\", \"w1\"]}},\n"
      << "  \"labels\": {\"w0\": [\"link_up\"], \"w1\": [\"link_up\"]},\n"
      << "  \"agents\": {" << agent_json(1, "r1", kR1Start) << "},\n"
      << "  \"zones\": {\"dock_1\": " << bounds_json(kDock1)
      << ", \"dock_2\": " << bounds_json(kDock2)
      << ", \"pallet\": {\"worlds\": {\"w0\": " << bounds_json(kBayAisle2)
      << ", \"w1\": " << bounds_json(kBayAisle3) << "}}}\n"
      << "}\n";
  return out.str();
}

std::string snapshot_fleet()
{
  // r2's relation is the discrete partition: it has been down the aisles and
  // tells the worlds apart. Same map, same zones, different knowledge.
  std::ostringstream out;
  out << "{\n"
      << "  \"worlds\": [\"w0\", \"w1\"],\n"
      << "  \"designated\": [\"w0\"],\n"
      << "  \"relations\": {\n"
      << "    \"r1\": {\"w0\": [\"w0\", \"w1\"], \"w1\": [\"w0\", \"w1\"]},\n"
      << "    \"r2\": {\"w0\": [\"w0\"], \"w1\": [\"w1\"]}\n"
      << "  },\n"
      << "  \"labels\": {\"w0\": [\"link_up\"], \"w1\": [\"link_up\"]},\n"
      << "  \"agents\": {" << agent_json(1, "r1", kR1Start) << ", "
      << agent_json(2, "r2", kR2Start) << "},\n"
      << "  \"zones\": {\"dock_1\": " << bounds_json(kDock1)
      << ", \"dock_2\": " << bounds_json(kDock2)
      << ", \"pallet\": {\"worlds\": {\"w0\": " << bounds_json(kBayAisle2)
      << ", \"w1\": " << bounds_json(kBayAisle3) << "}}}\n"
      << "}\n";
  return out.str();
}

// ---------------------------------------------------------------------------

std::vector<Case> cases()
{
  return {
    {"unknown-is-not-free", false, "docks", 1, "dock_1", false, "",
      "no route: the east corridor has never been observed"},

    {"after-sensing", true, "docks", 1, "dock_1", false, "",
      "a route: the same fixed point completes once that corridor reads free"},

    {"ontic-pallet", true, "pallet", 1, "pallet", false, "",
      "a route to the bay the designated world puts the pallet in"},

    {"epistemic-pallet", true, "pallet", 1, "pallet", true, "",
      "a route through the cells that settle which bay it is"},

    {"second-robot-knows", true, "fleet", 2, "pallet", true, "",
      "no sensing: r2 already tells the two worlds apart"},

    {"safety-behind-the-link", true, "docks", 1, "dock_2", false,
      R"({"connective":"and","formulas":["free","link_up"]})",
      "a route, and none at all in a world where the link is down"},
  };
}

std::string snapshot_for(const Case & c)
{
  if (c.snapshot == "docks") {return snapshot_docks();}
  if (c.snapshot == "pallet") {return snapshot_pallet();}
  if (c.snapshot == "fleet") {return snapshot_fleet();}
  throw std::invalid_argument("no snapshot named " + c.snapshot);
}

// ---------------------------------------------------------------------------

Outcome run(const Case & c)
{
  Outcome outcome;

  const auto grid = build_grid(c.east_corridor_observed);
  const auto state = classify(grid);
  const auto graph = build_graph(state);

  const auto snapshot = mu_path_planner::parse_snapshot(snapshot_for(c), grid_info());
  if (!snapshot.ok) {
    outcome.error = snapshot.error;
    return outcome;
  }

  mu_path_planner::QuerySpec spec;
  spec.agent_id = c.agent_id;
  spec.goal_zone = c.goal_zone;
  spec.safety_formula_json = c.safety_formula_json;
  spec.require_epistemic_goal = c.require_epistemic_goal;
  // Six cells at ten centimetres is sixty: a lidar resolves a bay from the
  // aisle it opens onto, not only from on top of it.
  spec.sensor_range_cells = 6;

  const auto query = mu_path_planner::resolve_query(snapshot, graph, state, spec);
  if (!query.ok) {
    outcome.error = query.error;
    return outcome;
  }

  outcome.start = query.start;
  outcome.goal_cells = query.goal.size();
  outcome.known_goal_cells = query.known_goal.size();
  outcome.disputed_cells = query.disputed.size();
  outcome.sensing_cells = query.sensing.size();
  outcome.safe_cells = query.safe.size();

  if (c.require_epistemic_goal) {
    auto result = mu_path_planner::mu_reach_epistemic(
      graph, query.goal, query.safe, query.sensing, query.start);
    outcome.winning_region = result.winning_region.size();
    outcome.iterations = result.iterations;
    outcome.path = std::move(result.path);
    outcome.sensing_waypoints = std::move(result.sensing_waypoints);
  } else {
    auto result = mu_path_planner::mu_reach(graph, query.goal, query.safe, query.start);
    outcome.winning_region = result.winning_region.size();
    outcome.iterations = result.iterations;
    outcome.path = std::move(result.path);
  }

  outcome.ok = true;
  return outcome;
}

}  // namespace warehouse_scenario
