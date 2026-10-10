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

// Fusion against canned grids. None of this needs a simulator, a robot or a
// ROS graph, which is why it is written before any of them exist.

#include <vector>

#include "epistemic_slam/map_fusion.hpp"
#include "gtest/gtest.h"

namespace
{

/// A grid from a literal, so a test reads like the map it is about.
/// -1 unobserved, 0 certainly free, 100 certainly occupied.
nav_msgs::msg::OccupancyGrid grid(
  unsigned width, unsigned height, const std::vector<std::int8_t> & cells)
{
  nav_msgs::msg::OccupancyGrid g;
  g.info.width = width;
  g.info.height = height;
  g.info.resolution = 0.05f;
  g.info.origin.orientation.w = 1.0;
  g.data = cells;
  return g;
}

constexpr std::int8_t U = -1;   // unobserved
constexpr std::int8_t F = 0;    // free
constexpr std::int8_t O = 100;  // occupied

}  // namespace

TEST(Classify, UnobservedIsNotFree)
{
  // The distinction the whole approach rests on. A planner that treats these
  // alike will drive a robot into a corridor nobody has looked at.
  EXPECT_EQ(epistemic_slam::classify(U), epistemic_slam::CellClass::Unknown);
  EXPECT_EQ(epistemic_slam::classify(F), epistemic_slam::CellClass::Free);
}

TEST(Classify, SeenButUndecidedIsAlsoUnknown)
{
  // Between the thresholds the sensor has produced no answer. Rounding it to
  // the nearest certainty would invent knowledge.
  EXPECT_EQ(epistemic_slam::classify(50), epistemic_slam::CellClass::Unknown);
  EXPECT_EQ(epistemic_slam::classify(25), epistemic_slam::CellClass::Unknown);
  EXPECT_EQ(epistemic_slam::classify(65), epistemic_slam::CellClass::Unknown);

  EXPECT_EQ(epistemic_slam::classify(24), epistemic_slam::CellClass::Free);
  EXPECT_EQ(epistemic_slam::classify(66), epistemic_slam::CellClass::Occupied);
}

TEST(Coverage, CountsWhatWasObservedNotWhatIsFree)
{
  const auto g = grid(2, 2, {U, F, O, U});

  const auto observed = epistemic_slam::coverage(g);
  EXPECT_FALSE(observed[0]);
  EXPECT_TRUE(observed[1]);
  EXPECT_TRUE(observed[2]);   // occupied is observed
  EXPECT_FALSE(observed[3]);

  EXPECT_DOUBLE_EQ(epistemic_slam::coverage_fraction(g), 0.5);
}

TEST(Fuse, RefusesGridsOfDifferentShape)
{
  const auto result = epistemic_slam::fuse(grid(2, 2, {U, U, U, U}), grid(2, 1, {U, U}));

  EXPECT_FALSE(result.ok);
  EXPECT_NE(result.error.find("differ in size"), std::string::npos) << result.error;
}

TEST(Fuse, RefusesGridsWithDifferentOrigins)
{
  auto moved = grid(2, 2, {U, U, U, U});
  moved.info.origin.position.x = 1.0;

  const auto result = epistemic_slam::fuse(grid(2, 2, {U, U, U, U}), moved);

  EXPECT_FALSE(result.ok);
  EXPECT_NE(result.error.find("registration"), std::string::npos) << result.error;
}

TEST(Fuse, EachRobotLearnsWhatTheOtherSaw)
{
  //   a saw the left column, b saw the right column.
  const auto a = grid(2, 2, {F, U, O, U});
  const auto b = grid(2, 2, {U, O, U, F});

  const auto result = epistemic_slam::fuse(a, b);
  ASSERT_TRUE(result.ok) << result.error;

  // The merged map carries both halves.
  EXPECT_EQ(result.merged.data[0], F);
  EXPECT_EQ(result.merged.data[1], O);
  EXPECT_EQ(result.merged.data[2], O);
  EXPECT_EQ(result.merged.data[3], F);

  // And each robot's gain is exactly the other's observations. This is what
  // gets announced to the epistemic state.
  EXPECT_EQ(result.newly_known_to_a, (std::vector<std::size_t>{1, 3}));
  EXPECT_EQ(result.newly_known_to_b, (std::vector<std::size_t>{0, 2}));
  EXPECT_TRUE(result.conflicts.empty());
}

TEST(Fuse, CellsNeitherRobotSawStayUnknown)
{
  const auto result = epistemic_slam::fuse(grid(1, 2, {F, U}), grid(1, 2, {F, U}));
  ASSERT_TRUE(result.ok) << result.error;

  EXPECT_EQ(result.merged.data[1], U);
  EXPECT_TRUE(result.newly_known_to_a.empty());
  EXPECT_TRUE(result.newly_known_to_b.empty());
}

TEST(Fuse, AgreeingReadingsAreNotAConflict)
{
  // Both say free, with different confidence. Ordinary, and not worth
  // reporting; the more confident reading survives.
  const auto result = epistemic_slam::fuse(grid(1, 1, {20}), grid(1, 1, {2}));
  ASSERT_TRUE(result.ok) << result.error;

  EXPECT_TRUE(result.conflicts.empty());
  EXPECT_EQ(result.merged.data[0], 2);
}

TEST(Fuse, FreeAgainstOccupiedIsReportedAndResolved)
{
  // One robot drove through it, the other saw an obstacle. Something has
  // changed or someone is wrong, and the fleet needs to know which cell.
  const auto result = epistemic_slam::fuse(grid(1, 2, {F, F}), grid(1, 2, {O, F}));
  ASSERT_TRUE(result.ok) << result.error;

  ASSERT_EQ(result.conflicts.size(), 1u);
  EXPECT_EQ(result.conflicts[0].index, 0u);
  EXPECT_EQ(result.conflicts[0].value_a, F);
  EXPECT_EQ(result.conflicts[0].value_b, O);

  // Still merged rather than left as a hole: a disputed cell can be
  // re-observed, a missing one cannot even be planned around.
  EXPECT_TRUE(epistemic_slam::is_known(result.merged.data[0]));
}

TEST(Fuse, IsSymmetricInWhatItReports)
{
  const auto a = grid(2, 1, {F, U});
  const auto b = grid(2, 1, {U, O});

  const auto forward = epistemic_slam::fuse(a, b);
  const auto backward = epistemic_slam::fuse(b, a);
  ASSERT_TRUE(forward.ok);
  ASSERT_TRUE(backward.ok);

  EXPECT_EQ(forward.merged.data, backward.merged.data);
  EXPECT_EQ(forward.newly_known_to_a, backward.newly_known_to_b);
  EXPECT_EQ(forward.newly_known_to_b, backward.newly_known_to_a);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

// ---------------------------------------------------------------------------
// Merge rules, on a map that has gone stale
//
// Two cells. Both robots held both as the shift map had them at time 0: the
// first free, the second occupied. Then the first was staged and the second
// cleared. The receiver saw the first change at time 10, the sender the
// second at time 20, and neither saw the other's.
// ---------------------------------------------------------------------------

namespace
{

struct Stale
{
  std::vector<std::int8_t> receiver{O, O};
  std::vector<double> receiver_at{10.0, 0.0};
  std::vector<std::int8_t> sender{F, F};
  std::vector<double> sender_at{0.0, 20.0};
  std::vector<std::size_t> cells{0, 1};
};

}  // namespace

TEST(Merge, ConfidenceNeverRepairsAStaleMap)
{
  // 0 and 100 are equally confident, the tie keeps the receiver's, and the
  // receiver keeps the cell the sender saw cleared.
  Stale s;
  const auto m = epistemic_slam::merge_cells(
    s.receiver, s.receiver_at, s.sender, s.sender_at, s.cells,
    epistemic_slam::MergeRule::Confidence);
  EXPECT_EQ(s.receiver, (std::vector<std::int8_t>{O, O}));
  EXPECT_EQ(m.changed, 0u);
  EXPECT_EQ(m.kept, 2u);
}

TEST(Merge, OverwriteSpreadsTheSendersStaleCell)
{
  // The receiver's fresh reading of the first cell is replaced by the
  // sender's stale one.
  Stale s;
  const auto m = epistemic_slam::merge_cells(
    s.receiver, s.receiver_at, s.sender, s.sender_at, s.cells,
    epistemic_slam::MergeRule::Overwrite);
  EXPECT_EQ(s.receiver, (std::vector<std::int8_t>{F, F}));
  EXPECT_EQ(m.changed, 2u);
}

TEST(Merge, RecencyTakesEachCellFromWhoeverSawItLast)
{
  Stale s;
  const auto m = epistemic_slam::merge_cells(
    s.receiver, s.receiver_at, s.sender, s.sender_at, s.cells,
    epistemic_slam::MergeRule::Recency);
  EXPECT_EQ(s.receiver, (std::vector<std::int8_t>{O, F}));
  EXPECT_EQ(m.changed, 1u);
  EXPECT_EQ(m.kept, 1u);
  // The cell taken keeps the time it was observed, not the time it arrived.
  EXPECT_DOUBLE_EQ(s.receiver_at[1], 20.0);
}

TEST(Merge, RecencyWithoutTimesKeepsTheReceiver)
{
  Stale s;
  std::vector<double> none;
  epistemic_slam::merge_cells(
    s.receiver, none, s.sender, none, s.cells, epistemic_slam::MergeRule::Recency);
  EXPECT_EQ(s.receiver, (std::vector<std::int8_t>{O, O}));
}

TEST(Merge, UnobservedCellsAreLearnedUnderEveryRule)
{
  for (const auto rule : {epistemic_slam::MergeRule::Confidence,
      epistemic_slam::MergeRule::Overwrite, epistemic_slam::MergeRule::Recency})
  {
    std::vector<std::int8_t> receiver{U, F};
    std::vector<double> receiver_at{0.0, 0.0};
    const std::vector<std::int8_t> sender{O, U};
    const std::vector<double> sender_at{5.0, 0.0};
    const auto m = epistemic_slam::merge_cells(
      receiver, receiver_at, sender, sender_at, {0, 1}, rule);
    EXPECT_EQ(receiver, (std::vector<std::int8_t>{O, F})) << epistemic_slam::to_string(rule);
    EXPECT_EQ(m.learned, 1u);
    EXPECT_EQ(m.considered, 1u);
  }
}
