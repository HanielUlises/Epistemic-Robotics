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

// The line-of-sight check the beacon's observability is measured with.

#include <gtest/gtest.h>

#include "coordinated_attack_demo/common.hpp"

namespace
{

/// 10 x 10 m, all free, at 0.1 m.
pass_through::Grid open_floor()
{
  pass_through::Grid g;
  g.width = 100;
  g.height = 100;
  g.resolution = 0.1;
  g.cells.assign(100 * 100, 0);
  return g;
}

void wall(pass_through::Grid & g, int column, int row0, int row1)
{
  for (int r = row0; r <= row1; ++r) {
    g.cells[r * g.width + column] = 100;
  }
}

}  // namespace

TEST(LineOfSight, OpenFloorIsClear)
{
  const auto g = open_floor();
  EXPECT_TRUE(coordinated_attack::line_of_sight(g, 1.0, 1.0, 9.0, 9.0));
}

TEST(LineOfSight, AWallBetweenBlocks)
{
  auto g = open_floor();
  wall(g, 50, 0, 99);
  EXPECT_FALSE(coordinated_attack::line_of_sight(g, 1.0, 5.0, 9.0, 5.0));
}

TEST(LineOfSight, AGapInTheWallIsASightLine)
{
  auto g = open_floor();
  wall(g, 50, 0, 44);
  wall(g, 50, 56, 99);
  EXPECT_TRUE(coordinated_attack::line_of_sight(g, 1.0, 5.0, 9.0, 5.0));
  EXPECT_FALSE(coordinated_attack::line_of_sight(g, 1.0, 1.0, 9.0, 1.0));
}

TEST(LineOfSight, TheTargetItselfDoesNotBlock)
{
  // The beacon's post is drawn as occupied; looking at it is not blocked by it.
  auto g = open_floor();
  g.cells[50 * g.width + 50] = 100;
  EXPECT_TRUE(coordinated_attack::line_of_sight(g, 1.0, 5.05, 5.05, 5.05));
}

TEST(LineOfSight, OffTheGridIsNotSeen)
{
  const auto g = open_floor();
  EXPECT_FALSE(coordinated_attack::line_of_sight(g, 1.0, 1.0, 12.0, 1.0, 0.0));
}
