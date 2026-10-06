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

#include <gtest/gtest.h>

#include <random>
#include <string>
#include <unordered_set>
#include <vector>

#include "mu_path_planner/mu_calculus.hpp"
#include "pass_through_demo/reach.hpp"

using pass_through::Box;
using pass_through::Grid;

namespace
{

// A 7 x 5 floor, 1 m cells. Row 2 is a wall with three one-cell gaps, the
// bays t1, t2 and t3; the carrier starts south of it and the dock is north.
//
//   row 4   . . . D . . .
//   row 3   . . . . . . .
//   row 2   # 1 # 2 # 3 #
//   row 1   . . . . . . .
//   row 0   . . . C . . .
Grid floor_with_bays()
{
  Grid g;
  g.width = 7;
  g.height = 5;
  g.resolution = 1.0;
  g.cells.assign(35, 0);
  for (int c = 0; c < 7; ++c) {
    g.cells[2 * 7 + c] = (c == 1 || c == 3 || c == 5) ? -1 : 100;
  }
  return g;
}

std::map<std::string, Box> bays()
{
  return {
    {"t1", Box{1.0, 2.0, 2.0, 3.0}},
    {"t2", Box{3.0, 2.0, 4.0, 3.0}},
    {"t3", Box{5.0, 2.0, 6.0, 3.0}}};
}

// The model after the last announcement of the elimination branch: the carrier
// has been told t1 and t3 are shut. Three worlds, one designated; the carrier
// considers only w1 possible from it, and in w1 t2 is open. The scouts still
// cannot tell w1 from the others.
const char * kCarrierKnowsT2 = R"({
  "worlds": ["w0", "w1", "w2"],
  "designated": ["w1"],
  "labels": {"w0": ["open_t1"], "w1": ["open_t2"], "w2": ["open_t3"]},
  "relations": {
    "carrier": {"w0": ["w0"], "w1": ["w1"], "w2": ["w2"]},
    "west": {"w0": ["w0", "w1", "w2"], "w1": ["w0", "w1", "w2"], "w2": ["w0", "w1", "w2"]},
    "east": {"w0": ["w0", "w1", "w2"], "w1": ["w0", "w1", "w2"], "w2": ["w0", "w1", "w2"]}
  }
})";

// The initial model: everyone considers all three worlds.
const char * kNobodyKnows = R"({
  "worlds": ["w0", "w1", "w2"],
  "designated": ["w0", "w1", "w2"],
  "labels": {"w0": ["open_t1"], "w1": ["open_t2"], "w2": ["open_t3"]},
  "relations": {
    "carrier": {"w0": ["w0", "w1", "w2"], "w1": ["w0", "w1", "w2"], "w2": ["w0", "w1", "w2"]}
  }
})";

pass_through::SafeSet safe_for(const Grid & plan, const Grid * knowledge, const std::string & model)
{
  pass_through::SafeSetInput in;
  in.floorplan = &plan;
  in.knowledge = knowledge;
  in.bays = bays();
  in.agent = "carrier";
  in.model_json = model;
  in.inflation = 0.0;
  return pass_through::build_safe_set(in);
}

std::vector<char> goal_at(const Grid & g, std::size_t cell)
{
  std::vector<char> goal(g.size(), 0);
  goal[cell] = 1;
  return goal;
}

}  // namespace

TEST(SafeSet, NothingKnownLiftsNoBayAndTheDockIsOutOfReach)
{
  const auto plan = floor_with_bays();
  const auto safe = safe_for(plan, nullptr, kNobodyKnows);
  ASSERT_TRUE(safe.ok) << safe.error;
  EXPECT_TRUE(safe.lifted.empty());
  EXPECT_TRUE(safe.known_open.empty());

  const auto reach = pass_through::least_fixed_point(safe.graph, goal_at(plan, 4 * 7 + 3), safe.safe);
  EXPECT_FALSE(reach.contains(0 * 7 + 3)) << "the carrier reached the dock knowing nothing";
}

TEST(SafeSet, KnowingABayOpenLiftsThatBayAndNoOther)
{
  const auto plan = floor_with_bays();
  const auto safe = safe_for(plan, nullptr, kCarrierKnowsT2);
  ASSERT_TRUE(safe.ok) << safe.error;
  EXPECT_EQ(safe.lifted, std::vector<std::string>{"t2"});
  EXPECT_EQ(safe.known_open, std::vector<std::string>{"t2"});
  EXPECT_TRUE(safe.safe[2 * 7 + 3]);
  EXPECT_FALSE(safe.safe[2 * 7 + 1]);
  EXPECT_FALSE(safe.safe[2 * 7 + 5]);

  const auto reach = pass_through::least_fixed_point(safe.graph, goal_at(plan, 4 * 7 + 3), safe.safe);
  ASSERT_TRUE(reach.contains(0 * 7 + 3));
  const auto path = pass_through::descend(safe.graph, reach, 0 * 7 + 3);
  ASSERT_FALSE(path.empty());
  EXPECT_EQ(path.back(), 4u * 7 + 3);
  EXPECT_NE(std::find(path.begin(), path.end(), 2u * 7 + 3), path.end()) << "not through t2";
}

TEST(SafeSet, ABayObservedFreeIsSafeWithoutAnyKnowledgeOfTheModel)
{
  // The first disjunct: a bay the carrier's own map shows free is free,
  // whatever the model says. This is the scout's case after it has looked.
  const auto plan = floor_with_bays();
  Grid knowledge = pass_through::blank_like(plan, -1);
  knowledge.cells[2 * 7 + 5] = 0;
  const auto safe = safe_for(plan, &knowledge, kNobodyKnows);
  ASSERT_TRUE(safe.ok) << safe.error;
  EXPECT_TRUE(safe.safe[2 * 7 + 5]);
  EXPECT_TRUE(safe.lifted.empty()) << "observed free is not lifted: nothing had to be inferred";
}

TEST(SafeSet, AnObservedLoadIsNeverSafe)
{
  // Knowledge cannot drive through an obstacle the map shows. If the model
  // said t2 open and the map showed a load in it, the map wins: the formula
  // lifts unknown cells, not occupied ones.
  const auto plan = floor_with_bays();
  Grid knowledge = pass_through::blank_like(plan, -1);
  knowledge.cells[2 * 7 + 3] = 100;
  const auto safe = safe_for(plan, &knowledge, kCarrierKnowsT2);
  ASSERT_TRUE(safe.ok) << safe.error;
  EXPECT_FALSE(safe.safe[2 * 7 + 3]);
}

TEST(FixedPoint, AgreesWithKleeneIterationOnRandomFloors)
{
  // The semi-naive evaluation has to be the same fixed point mu_reach
  // computes, in the region and in the number of iterations.
  std::mt19937 rng(7);
  for (int trial = 0; trial < 60; ++trial) {
    const std::uint32_t w = 6 + rng() % 20, h = 6 + rng() % 20;
    mu_path_planner::OccupancyGraph graph;
    graph.width = w;
    graph.height = h;
    graph.obstacle.assign(w * h, false);
    for (std::size_t i = 0; i < w * h; ++i) {
      graph.obstacle[i] = rng() % 100 < 30;
    }
    graph.build_adjacency();

    std::vector<char> safe(w * h), goal(w * h, 0);
    std::unordered_set<mu_path_planner::CellIdx> safe_set, goal_set;
    for (std::size_t i = 0; i < w * h; ++i) {
      safe[i] = !graph.obstacle[i];
      if (safe[i]) {safe_set.insert(static_cast<mu_path_planner::CellIdx>(i));}
    }
    for (int g = 0; g < 2; ++g) {
      const std::size_t i = rng() % (w * h);
      if (safe[i]) {
        goal[i] = 1;
        goal_set.insert(static_cast<mu_path_planner::CellIdx>(i));
      }
    }
    if (goal_set.empty()) {continue;}

    const auto mine = pass_through::least_fixed_point(graph, goal, safe);
    const auto theirs = mu_path_planner::mu_reach(graph, goal_set, safe_set, 0);

    EXPECT_EQ(mine.size, theirs.winning_region.size()) << "trial " << trial;
    for (std::size_t i = 0; i < w * h; ++i) {
      EXPECT_EQ(mine.contains(i), theirs.winning_region.count(static_cast<mu_path_planner::CellIdx>(i)) > 0)
        << "trial " << trial << " cell " << i;
    }
    EXPECT_EQ(mine.iterations, theirs.iterations) << "trial " << trial;
  }
}

TEST(FixedPoint, DescentIsAShortestPath)
{
  mu_path_planner::OccupancyGraph graph;
  graph.width = 10;
  graph.height = 1;
  graph.obstacle.assign(10, false);
  graph.build_adjacency();
  std::vector<char> safe(10, 1), goal(10, 0);
  goal[9] = 1;
  const auto reach = pass_through::least_fixed_point(graph, goal, safe);
  const auto path = pass_through::descend(graph, reach, 0);
  ASSERT_EQ(path.size(), 10u);
  EXPECT_EQ(path.front(), 0u);
  EXPECT_EQ(path.back(), 9u);
}

TEST(Shortcut, ErosionKeepsOnlyCellsWhoseNeighboursAreAllSafe)
{
  Grid g;
  g.width = 5;
  g.height = 5;
  g.resolution = 1.0;
  g.cells.assign(25, 0);
  std::vector<char> safe(25, 1);
  safe[0] = 0;   // one unsafe corner
  const auto core = pass_through::erode(g, safe);
  EXPECT_FALSE(core[1 * 5 + 1]) << "a neighbour of the unsafe corner";
  EXPECT_TRUE(core[2 * 5 + 2]);
  EXPECT_TRUE(core[3 * 5 + 3]);
  EXPECT_FALSE(core[0 * 5 + 2]) << "the border has no neighbours outside the grid to vouch for it";
}
