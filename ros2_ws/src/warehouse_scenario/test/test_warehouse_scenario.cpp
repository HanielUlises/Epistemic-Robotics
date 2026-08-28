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
  const auto before = build_grid(Stage::FirstPass);
  const auto after = build_grid(Stage::AfterTheSweep);
  ASSERT_EQ(before.size(), after.size());

  std::size_t unknown_before = 0, unknown_after = 0, occupied_before = 0,
    occupied_after = 0;
  for (std::size_t i = 0; i < before.size(); ++i) {
    unknown_before += before[i] == kUnknown;
    unknown_after += after[i] == kUnknown;
    occupied_before += before[i] == kOccupied;
    occupied_after += after[i] == kOccupied;

    // A stage only ever settles cells; it never unsettles one and never
    // contradicts one. So wherever they differ, the first stage is the one
    // that had not looked -- and a cell it had looked at reads in the second
    // exactly as it read in the first.
    if (before[i] != after[i]) {
      EXPECT_EQ(before[i], kUnknown) << "a settled cell changed its mind";
      EXPECT_NE(after[i], kUnknown);
    }
  }

  // The sweep is the whole of the difference, and it settles a great deal:
  // the length of the building and every aisle off the service lane.
  EXPECT_GT(unknown_before, unknown_after);
  EXPECT_GT(occupied_after, occupied_before);

  // Neither stage has been everywhere -- a laser that drove the west corridor
  // and the lane has still not been into the cluttered north-east -- which is
  // the point: a map is what has been measured, not what is there.
  EXPECT_GT(unknown_after, 0u);

  // And underneath both, the building did not move: every cell either stage
  // has settled reads exactly as the floor the planner is entitled to. A rack
  // that shifted between stages would make the comparison worthless.
  const auto floor = inflate(build_floorplan(), kRobotRadiusM);
  for (std::size_t i = 0; i < floor.size(); ++i) {
    if (before[i] != kUnknown) {EXPECT_EQ(before[i], floor[i]);}
    if (after[i] != kUnknown) {EXPECT_EQ(after[i], floor[i]);}
  }
}

TEST(Warehouse, TheFloorPlanIsTheOneTheWorldHas)
{
  // Not a resemblance: these are the AWS world's own model poses, and the
  // rasteriser is only allowed to have put obstacles where they are.
  const auto floorplan = build_floorplan();
  ASSERT_EQ(floorplan.size(), static_cast<std::size_t>(kWidth) * kHeight);

  // Nothing is unknown in the floor plan itself; a stage is what adds that.
  for (const int8_t value : floorplan) {
    EXPECT_NE(value, kUnknown);
  }

  // The six rack rows: solid down their length, with the aisles between them
  // free the whole way.
  for (const double y : {1.02, -0.80, -2.60, -4.39, -6.31, -8.23}) {
    EXPECT_EQ(floorplan[cell_of(4.7, y - 0.4)], kOccupied)
      << "no rack at y = " << y;
  }
  for (const Box & aisle : {kBayAisle2, kBayAisle3, kBayAisle4}) {
    for (double x = aisle.min_x; x <= aisle.max_x; x += kResolution) {
      for (double y = aisle.min_y; y <= aisle.max_y; y += kResolution) {
        ASSERT_EQ(floorplan[cell_of(x, y)], kFree)
          << "the aisle is blocked at " << x << ", " << y;
      }
    }
  }

  // Behind the racks the world leaves 0.15 m of bare floor before the east
  // wall. On the building's own plan it is free, and it runs the length of
  // the rack block joining every aisle to every other.
  EXPECT_EQ(floorplan[cell_of(6.75, -2.14)], kFree);
  EXPECT_EQ(floorplan[cell_of(6.75, -1.20)], kFree);
}

TEST(Warehouse, TheGridThePlannerSeesIsTheFloorMinusTheRobot)
{
  // 0.15 m of floor is no corridor at all to something 0.21 m across, and the
  // grid the fixed point runs on has to say so -- otherwise it returns routes
  // behind the racks that no robot could drive. Growing the obstacles by the
  // radius is what closes it.
  const auto floorplan = build_floorplan();
  const auto grown = inflate(floorplan, kRobotRadiusM);

  EXPECT_EQ(grown[cell_of(6.75, -2.14)], kOccupied) << "the gap stayed open";
  EXPECT_EQ(grown[cell_of(6.75, -1.20)], kOccupied);

  // And it costs the aisles a robot's radius at each rack and no more, so
  // every bay is still floor the robot's centre may stand on.
  for (const Box & bay : {kBayAisle2, kBayAisle3, kBayAisle4}) {
    for (double x = bay.min_x; x <= bay.max_x; x += kResolution) {
      for (double y = bay.min_y; y <= bay.max_y; y += kResolution) {
        ASSERT_EQ(grown[cell_of(x, y)], kFree)
          << "the robot cannot stand at " << x << ", " << y;
      }
    }
  }

  // Growing obstacles never frees a cell.
  for (std::size_t i = 0; i < floorplan.size(); ++i) {
    if (floorplan[i] == kOccupied) {EXPECT_EQ(grown[i], kOccupied);}
  }
}

TEST(Warehouse, AStageIsWhatALaserCouldHaveReturned)
{
  // The stage is ray cast, not painted, and the test says so by casting one
  // ray of its own: a cell behind a rack is not observed from in front of it,
  // however close the two are.
  const auto floorplan = build_floorplan();
  const auto seen = observed_from(floorplan, {Point{kServiceLaneX, -2.14}},
      kLaserRangeM);

  EXPECT_TRUE(seen[cell_of(3.6, -2.14)]) << "down the aisle it is looking at";
  EXPECT_FALSE(seen[cell_of(3.6, -3.9)]) << "the next aisle over, behind a rack";
  EXPECT_FALSE(seen[cell_of(-3.5, -2.14)]) << "beyond the range of the laser";
}

TEST(Warehouse, AnUnobservedCellIsAnObstacleToTheFixedPoint)
{
  const auto state = classify(build_grid(Stage::FirstPass));
  const auto graph = build_graph(state);

  // The floor of aisle 4 is free in the world and unknown to a fleet that has
  // only driven the west corridor, and it is the second of those the fixed
  // point is entitled to act on.
  const CellIdx in_aisle = cell_of(4.3, -5.8);
  EXPECT_EQ(build_floorplan()[in_aisle], kFree);
  EXPECT_EQ(state[in_aisle], CellState::Unknown);
  EXPECT_TRUE(graph.obstacle[in_aisle]);

  const CellIdx in_corridor = cell_of(kR1Start.x, kR1Start.y);
  EXPECT_EQ(state[in_corridor], CellState::Free);
  EXPECT_FALSE(graph.obstacle[in_corridor]);
}

// ---------------------------------------------------------------------------
// Unknown is not free
// ---------------------------------------------------------------------------

TEST(Scenario, WithoutLookingDownTheAisleThereIsNoRouteIntoIt)
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

  // And the route it now finds runs up the west corridor -- the floor between
  // the two ends of the building, which is exactly what the first stage had
  // never measured and the sweep settled.
  EXPECT_TRUE(visits(after.path, Box{-4.6, -4.5, -2.5, 1.5}));
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
  // waypoint stands within a sensor's reach of one of the two bays -- the
  // same reach the query was resolved with, not a number picked to fit.
  const double reach = kSensorRangeCells * kResolution;
  for (const CellIdx cell : outcome.sensing_waypoints) {
    const Point at = point_of(cell);
    const auto within = [&at, reach](const Box & bay) {
        return at.x > bay.min_x - reach && at.x < bay.max_x + reach &&
               at.y > bay.min_y - reach && at.y < bay.max_y + reach;
      };
    EXPECT_TRUE(within(kBayAisle2) || within(kBayAisle3))
      << "sensing waypoint at " << at.x << ", " << at.y;
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
  EXPECT_TRUE(visits(outcome.path, kBayAisle4));
  EXPECT_GT(outcome.safe_cells, 0u);

  // Take the link down and the same query has nowhere safe to stand: not a
  // longer route, no route, and no cell in the safe set at all.
  Case down = case_named("safety-behind-the-link");
  down.name = "safety-behind-the-link-down";
  const auto grid = build_grid(down.stage);
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
  spec.sensor_range_cells = kSensorRangeCells;

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
