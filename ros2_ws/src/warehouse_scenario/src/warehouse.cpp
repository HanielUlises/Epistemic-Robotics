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

/// The zones every snapshot grounds, whatever else it disputes.
std::string common_zones_json()
{
  std::ostringstream out;
  out << "\"dock_south\": " << bounds_json(kDockSouth)
      << ", \"dock_north\": " << bounds_json(kDockNorth)
      << ", \"aisle_4\": " << bounds_json(kBayAisle4);
  return out.str();
}

/// The number of rays a stage is cast with.
///
/// At 2880 the angular step is an eighth of a degree, so two neighbouring rays
/// have not parted by a cell until 23 m out — further than the diagonal of the
/// building. Fewer rays and a stage acquires speckle: single unobserved cells
/// in the middle of observed floor, which the fixed point would then have to
/// route around for no reason on the ground.
constexpr int kRayCount = 2880;

}  // namespace

// ---------------------------------------------------------------------------
// The floor
// ---------------------------------------------------------------------------

std::vector<int8_t> build_floorplan()
{
  std::vector<int8_t> grid;
  grid.reserve(static_cast<std::size_t>(kWidth) * kHeight);

  int8_t value = kFree;
  for (std::size_t run = 0; run < kFloorplanRunCount; ++run) {
    grid.insert(grid.end(), kFloorplanRuns[run], value);
    value = value == kFree ? kOccupied : kFree;
  }

  if (grid.size() != static_cast<std::size_t>(kWidth) * kHeight) {
    throw std::runtime_error("the generated floor plan is not " +
            std::to_string(kWidth) + "x" + std::to_string(kHeight));
  }
  return grid;
}

std::vector<int8_t> inflate(const std::vector<int8_t> & floorplan, double radius_m)
{
  const auto reach = static_cast<int>(std::ceil(radius_m / kResolution));
  const double radius_cells = radius_m / kResolution;

  std::vector<int8_t> grown = floorplan;
  for (int row = 0; row < static_cast<int>(kHeight); ++row) {
    for (int col = 0; col < static_cast<int>(kWidth); ++col) {
      if (floorplan[static_cast<std::size_t>(row) * kWidth + col] != kOccupied) {
        continue;
      }
      // A disc, not a square: a square would clip the corner of every aisle
      // by more than the robot takes up and shorten the bays for no reason.
      for (int dr = -reach; dr <= reach; ++dr) {
        for (int dc = -reach; dc <= reach; ++dc) {
          if (dr * dr + dc * dc > radius_cells * radius_cells) {continue;}
          const int r = row + dr;
          const int c = col + dc;
          if (r < 0 || c < 0 || r >= static_cast<int>(kHeight) ||
            c >= static_cast<int>(kWidth))
          {
            continue;
          }
          grown[static_cast<std::size_t>(r) * kWidth + c] = kOccupied;
        }
      }
    }
  }
  return grown;
}

std::vector<Point> vantages(Stage stage)
{
  // Where a robot has stood, not where it would be convenient for it to have
  // stood: every one of these is on free floor, and the fleet could have
  // driven from one to the next.
  std::vector<Point> out;

  // What both stages have: each robot has looked around the end of the
  // building it started at, and no further. r1 came on shift at the shipping
  // floor in the south, r2 at receiving in the north, and neither has yet had
  // a reason to walk the length of the warehouse.
  for (double y = -9.4; y < -7.9; y += 0.7) {
    out.push_back(Point{kR1Start.x, y});
  }
  for (double y = 5.3; y < 7.6; y += 0.7) {
    out.push_back(Point{kR2Start.x, y});
  }
  if (stage == Stage::FirstPass) {
    return out;
  }

  // The sweep. Up the west corridor first, between the shelf on the west wall
  // and the clutter down the middle, which is what joins the two ends of the
  // building into one known floor.
  for (double y = -8.2; y < 5.4; y += 1.2) {
    out.push_back(Point{-3.5, y});
  }

  // Then along the south wall, which is how a robot gets from that corridor
  // to the rack block without crossing it.
  for (double x = -2.5; x < 2.4; x += 1.2) {
    out.push_back(Point{x, -9.6});
  }

  // And up the service lane, which is the only floor the aisles open onto and
  // therefore the only floor a laser can see down them from.
  for (double y = -8.6; y < 0.7; y += 1.0) {
    out.push_back(Point{kServiceLaneX, y});
  }
  return out;
}

std::vector<bool> observed_from(
  const std::vector<int8_t> & floorplan, const std::vector<Point> & from,
  double range_m)
{
  std::vector<bool> seen(floorplan.size(), false);
  const auto steps = static_cast<int>(range_m / kResolution);

  for (const Point & at : from) {
    const auto origin_col = static_cast<int>((at.x - kOriginX) / kResolution);
    const auto origin_row = static_cast<int>((at.y - kOriginY) / kResolution);
    if (origin_col < 0 || origin_row < 0 ||
      origin_col >= static_cast<int>(kWidth) ||
      origin_row >= static_cast<int>(kHeight))
    {
      throw std::out_of_range("a vantage point is off the map");
    }
    seen[static_cast<std::size_t>(origin_row) * kWidth + origin_col] = true;

    for (int ray = 0; ray < kRayCount; ++ray) {
      const double angle = 2.0 * M_PI * ray / kRayCount;
      const double dx = std::cos(angle);
      const double dy = std::sin(angle);

      for (int step = 1; step <= steps; ++step) {
        const int col = origin_col + static_cast<int>(dx * step);
        const int row = origin_row + static_cast<int>(dy * step);
        if (col < 0 || row < 0 || col >= static_cast<int>(kWidth) ||
          row >= static_cast<int>(kHeight))
        {
          break;
        }
        const auto cell = static_cast<std::size_t>(row) * kWidth + col;
        seen[cell] = true;
        // A return comes off the first thing the ray meets, and nothing
        // behind it is measured. That is what leaves an aisle unknown.
        if (floorplan[cell] == kOccupied) {
          break;
        }
      }
    }
  }
  return seen;
}

std::vector<int8_t> build_grid(Stage stage)
{
  const auto floorplan = build_floorplan();
  const auto seen = observed_from(floorplan, vantages(stage), kLaserRangeM);

  auto grid = inflate(floorplan, kRobotRadiusM);
  for (std::size_t i = 0; i < grid.size(); ++i) {
    if (!seen[i]) {
      grid[i] = kUnknown;
    }
  }
  return grid;
}

CellIdx cell_of(double x, double y)
{
  const auto col = static_cast<int64_t>((x - kOriginX) / kResolution);
  const auto row = static_cast<int64_t>((y - kOriginY) / kResolution);
  if (col < 0 || row < 0 || col >= kWidth || row >= kHeight) {
    throw std::out_of_range("point is off the map");
  }
  return static_cast<CellIdx>(row) * kWidth + static_cast<CellIdx>(col);
}

Point point_of(CellIdx cell)
{
  const uint32_t col = cell % kWidth;
  const uint32_t row = cell / kWidth;
  return Point{kOriginX + (col + 0.5) * kResolution,
    kOriginY + (row + 0.5) * kResolution};
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
  info.origin_x = kOriginX;
  info.origin_y = kOriginY;
  return info;
}

// ---------------------------------------------------------------------------
// What the robots know
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
      << "  \"zones\": {" << common_zones_json() << "}\n"
      << "}\n";
  return out.str();
}

std::string snapshot_pallet()
{
  // r1 cannot tell w0 from w1, and the two disagree about which aisle is the
  // pallet's. Every cell they disagree about is a cell sensing is for.
  std::ostringstream out;
  out << "{\n"
      << "  \"worlds\": [\"w0\", \"w1\"],\n"
      << "  \"designated\": [\"w0\"],\n"
      << "  \"relations\": {\"r1\": {\"w0\": [\"w0\", \"w1\"], "
      << "\"w1\": [\"w0\", \"w1\"]}},\n"
      << "  \"labels\": {\"w0\": [\"link_up\"], \"w1\": [\"link_up\"]},\n"
      << "  \"agents\": {" << agent_json(1, "r1", kR1Start) << "},\n"
      << "  \"zones\": {" << common_zones_json()
      << ", \"pallet\": {\"worlds\": {\"w0\": " << bounds_json(kBayAisle2)
      << ", \"w1\": " << bounds_json(kBayAisle3) << "}}}\n"
      << "}\n";
  return out.str();
}

std::string snapshot_fleet()
{
  // r2's relation is the discrete partition: it has driven the lane past both
  // aisle mouths and tells the worlds apart. Same map, same zones, different
  // knowledge.
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
      << "  \"zones\": {" << common_zones_json()
      << ", \"pallet\": {\"worlds\": {\"w0\": " << bounds_json(kBayAisle2)
      << ", \"w1\": " << bounds_json(kBayAisle3) << "}}}\n"
      << "}\n";
  return out.str();
}

// ---------------------------------------------------------------------------

std::vector<Case> cases()
{
  return {
    {"unknown-is-not-free", Stage::FirstPass, "docks", 1, "dock_north", false,
      "", "no route: the floor between the two ends has never been measured"},

    {"after-sensing", Stage::AfterTheSweep, "docks", 1, "dock_north", false, "",
      "a route: the same fixed point completes once that floor reads free"},

    {"ontic-pallet", Stage::AfterTheSweep, "pallet", 1, "pallet", false, "",
      "a route to the aisle the designated world puts the pallet in"},

    {"epistemic-pallet", Stage::AfterTheSweep, "pallet", 1, "pallet", true, "",
      "a route through the cells that settle which aisle it is"},

    {"second-robot-knows", Stage::AfterTheSweep, "fleet", 2, "pallet", true, "",
      "no sensing: r2 already tells the two worlds apart"},

    {"safety-behind-the-link", Stage::AfterTheSweep, "docks", 1, "aisle_4",
      false, R"({"connective":"and","formulas":["free","link_up"]})",
      "a route into the deep aisle, and none at all with the link down"},
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

  const auto grid = build_grid(c.stage);
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
  spec.sensor_range_cells = kSensorRangeCells;

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
