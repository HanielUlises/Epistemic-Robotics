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
// What the warehouse scenario is there to show, asserted rather than looked
// at in a picture. Every case runs the planner's own resolve_query and its
// own fixed point; none of them needs ROS, a simulator or a robot.

#include <gtest/gtest.h>

#include <algorithm>
#include <string>

#include "warehouse_scenario/warehouse.hpp"

using namespace warehouse_scenario;

namespace
{

Case case_named(const std::string & name)
{
  for (const auto & c : cases()) {
    if (c.name == name) {return c;}
  }
  throw std::invalid_argument("no case named " + name);
}

/// Does the route ever set foot in the box?
bool visits(const std::vector<CellIdx> & cells, const Box & box)
{
  return std::any_of(cells.begin(), cells.end(), [&box](CellIdx cell) {
             const Point at = point_of(cell);
             return at.x >= box.min_x && at.x <= box.max_x &&
             at.y >= box.min_y && at.y <= box.max_y;
           });
}

}  // namespace

// ---------------------------------------------------------------------------
// The floor
// ---------------------------------------------------------------------------

TEST(Warehouse, TheTwoStagesDifferOnlyInWhatWasObserved)
{
  const auto before = build_grid(false);
  const auto after = build_grid(true);
  ASSERT_EQ(before.size(), after.size());

  std::size_t unknown_before = 0, unknown_after = 0, occupied_before = 0,
    occupied_after = 0;
  for (std::size_t i = 0; i < before.size(); ++i) {
    unknown_before += before[i] == kUnknown;
    unknown_after += after[i] == kUnknown;
    occupied_before += before[i] == kOccupied;
    occupied_after += after[i] == kOccupied;

    // Wherever the stages disagree, the disagreement is unknown-versus-free.
    // A rack that moved between stages would make the comparison worthless.
    if (before[i] != after[i]) {
      EXPECT_EQ(before[i], kUnknown);
      EXPECT_EQ(after[i], kFree);
    }
  }

  EXPECT_GT(unknown_before, 0u);
  EXPECT_EQ(unknown_after, 0u);
  EXPECT_EQ(occupied_before, occupied_after);
}

TEST(Warehouse, AnUnobservedCellIsAnObstacleToTheFixedPoint)
{
  const auto state = classify(build_grid(false));
  const auto graph = build_graph(state);

  const CellIdx in_corridor = cell_of(
    (kEastCorridorX0 + kEastCorridorX1) / 2.0, 7.0);
  EXPECT_EQ(state[in_corridor], CellState::Unknown);
  EXPECT_TRUE(graph.obstacle[in_corridor]);

  const CellIdx in_aisle = cell_of(2.0, 1.6);
  EXPECT_EQ(state[in_aisle], CellState::Free);
  EXPECT_FALSE(graph.obstacle[in_aisle]);
}

// ---------------------------------------------------------------------------
// Unknown is not free
// ---------------------------------------------------------------------------

TEST(Scenario, WithoutObservingTheCorridorThereIsNoRouteToTheDock)
{
  const Outcome outcome = run(case_named("unknown-is-not-free"));
  ASSERT_TRUE(outcome.ok) << outcome.error;

  // The goal is grounded and the fixed point ran; it simply does not reach
  // the robot, which is a different thing from the query failing.
  EXPECT_GT(outcome.goal_cells, 0u);
  EXPECT_GT(outcome.winning_region, 0u);
  EXPECT_TRUE(outcome.path.empty());
}

TEST(Scenario, ObservingItIsTheWholeDifference)
{
  const Outcome before = run(case_named("unknown-is-not-free"));
  const Outcome after = run(case_named("after-sensing"));
  ASSERT_TRUE(after.ok) << after.error;

  EXPECT_TRUE(before.path.empty());
  EXPECT_FALSE(after.path.empty());
  EXPECT_EQ(before.start, after.start);
  EXPECT_EQ(before.goal_cells, after.goal_cells);

  // And the route it now finds goes through the corridor that was unknown.
  EXPECT_TRUE(visits(after.path,
    Box{kEastCorridorX0, 0.0, kEastCorridorX1, kHeightM}));
}

// ---------------------------------------------------------------------------
// Being there and knowing you are there
// ---------------------------------------------------------------------------

TEST(Scenario, AnOnticGoalNeitherSensesNorCares)
{
  const Outcome outcome = run(case_named("ontic-pallet"));
  ASSERT_TRUE(outcome.ok) << outcome.error;

  // w0 is the designated world and puts the pallet in aisle 2, so that is
  // where the robot is sent, and it is sent without settling anything.
  ASSERT_FALSE(outcome.path.empty());
  EXPECT_TRUE(visits(outcome.path, kBayAisle2));
  EXPECT_TRUE(outcome.sensing_waypoints.empty());

  // The disagreement is there all the same: it is simply not this goal's
  // problem. Nothing the robot knows tells it which bay it is standing in.
  EXPECT_GT(outcome.disputed_cells, 0u);
  EXPECT_EQ(outcome.known_goal_cells, 0u);
}

TEST(Scenario, AnEpistemicGoalHasToSettleWhichBayItIs)
{
  const Outcome outcome = run(case_named("epistemic-pallet"));
  ASSERT_TRUE(outcome.ok) << outcome.error;

  EXPECT_GT(outcome.disputed_cells, 0u);
  EXPECT_GT(outcome.sensing_cells, 0u);
  ASSERT_FALSE(outcome.path.empty());
  EXPECT_FALSE(outcome.sensing_waypoints.empty());

  // Sensing is only worth spending where the worlds disagree, so every
  // waypoint stands within reach of one of the two bays.
  for (const CellIdx cell : outcome.sensing_waypoints) {
    const Point at = point_of(cell);
    const bool near_a_bay =
      (at.x > kBayAisle2.min_x - 1.0 && at.x < kBayAisle2.max_x + 1.0 &&
      at.y > kBayAisle2.min_y - 1.0 && at.y < kBayAisle2.max_y + 1.0) ||
      (at.x > kBayAisle3.min_x - 1.0 && at.x < kBayAisle3.max_x + 1.0 &&
      at.y > kBayAisle3.min_y - 1.0 && at.y < kBayAisle3.max_y + 1.0);
    EXPECT_TRUE(near_a_bay) << "sensing waypoint at " << at.x << ", " << at.y;
  }
}

TEST(Scenario, KnowingAlreadyIsCheaperThanFindingOut)
{
  const Outcome r1 = run(case_named("epistemic-pallet"));
  const Outcome r2 = run(case_named("second-robot-knows"));
  ASSERT_TRUE(r2.ok) << r2.error;

  // Same map, same zones, same goal. r2 tells w0 from w1, so for r2 nothing
  // is in dispute and nothing has to be sensed; for r1 both are.
  EXPECT_GT(r1.disputed_cells, 0u);
  EXPECT_EQ(r2.disputed_cells, 0u);
  EXPECT_FALSE(r1.sensing_waypoints.empty());
  EXPECT_TRUE(r2.sensing_waypoints.empty());

  EXPECT_GT(r2.known_goal_cells, 0u);
  ASSERT_FALSE(r2.path.empty());
  EXPECT_TRUE(visits(r2.path, kBayAisle2));
}

// ---------------------------------------------------------------------------
// A safety constraint is a formula, not a mask
// ---------------------------------------------------------------------------

TEST(Scenario, ASafetyConstraintIsEvaluatedAgainstTheModel)
{
  const Outcome outcome = run(case_named("safety-behind-the-link"));
  ASSERT_TRUE(outcome.ok) << outcome.error;

  // free ∧ link_up, and the link is up in the only world there is.
  ASSERT_FALSE(outcome.path.empty());
  EXPECT_TRUE(visits(outcome.path, kDock2));
  EXPECT_GT(outcome.safe_cells, 0u);

  // Take the link down and the same query has nowhere safe to stand: not a
  // longer route, no route, and no cell in the safe set at all.
  Case down = case_named("safety-behind-the-link");
  down.name = "safety-behind-the-link-down";
  const auto grid = build_grid(down.east_corridor_observed);
  const auto state = classify(grid);
  const auto graph = build_graph(state);

  std::string text = snapshot_for(down);
  const std::string label = "\"link_up\"";
  const auto at = text.find(label);
  ASSERT_NE(at, std::string::npos);
  text.replace(at, label.size(), "\"link_down\"");

  const auto snapshot = mu_path_planner::parse_snapshot(text, grid_info());
  ASSERT_TRUE(snapshot.ok) << snapshot.error;

  mu_path_planner::QuerySpec spec;
  spec.agent_id = down.agent_id;
  spec.goal_zone = down.goal_zone;
  spec.safety_formula_json = down.safety_formula_json;
  spec.require_epistemic_goal = false;
  spec.sensor_range_cells = 6;

  const auto query = mu_path_planner::resolve_query(snapshot, graph, state, spec);
  ASSERT_TRUE(query.ok) << query.error;
  EXPECT_TRUE(query.safe.empty());

  const auto result = mu_path_planner::mu_reach(graph, query.goal, query.safe, query.start);
  EXPECT_TRUE(result.path.empty());
}

// ---------------------------------------------------------------------------
// The cases themselves
// ---------------------------------------------------------------------------

TEST(Scenario, EveryCaseResolvesAndIsNamedOnce)
{
  std::vector<std::string> names;
  for (const auto & c : cases()) {
    const Outcome outcome = run(c);
    EXPECT_TRUE(outcome.ok) << c.name << ": " << outcome.error;
    names.push_back(c.name);
  }
  std::sort(names.begin(), names.end());
  EXPECT_EQ(std::unique(names.begin(), names.end()), names.end());
}
