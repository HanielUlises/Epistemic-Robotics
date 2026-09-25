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

#include <cstdio>
#include <fstream>
#include <string>

#include "pass_through_demo/grid.hpp"

using pass_through::Box;
using pass_through::Grid;
using pass_through::Verdict;

namespace
{

Grid square(std::int8_t value)
{
  Grid g;
  g.width = 10;
  g.height = 10;
  g.resolution = 0.1;
  g.cells.assign(100, value);
  return g;
}

}  // namespace

TEST(Region, AllObservedFreeIsClear)
{
  const auto g = square(0);
  EXPECT_EQ(pass_through::read_region(g, Box{0.2, 0.2, 0.7, 0.7}).verdict, Verdict::Clear);
}

TEST(Region, OneUnobservedCellLeavesItUndecided)
{
  auto g = square(0);
  g.cells[4 * 10 + 4] = -1;
  const auto r = pass_through::read_region(g, Box{0.2, 0.2, 0.7, 0.7});
  EXPECT_EQ(r.verdict, Verdict::Unknown);
  EXPECT_EQ(r.unknown, 1u);
}

TEST(Region, OneOccupiedCellBlocksItWhateverElseIsUnknown)
{
  auto g = square(-1);
  g.cells[4 * 10 + 4] = 100;
  EXPECT_EQ(pass_through::read_region(g, Box{0.2, 0.2, 0.7, 0.7}).verdict, Verdict::Blocked);
}

TEST(Region, SeenButUnsettledIsNotFree)
{
  // The band between the thresholds is a cell seen without being settled.
  // Calling it free would hand an agent knowledge it does not have.
  auto g = square(0);
  g.cells[4 * 10 + 4] = 50;
  EXPECT_EQ(pass_through::read_region(g, Box{0.2, 0.2, 0.7, 0.7}).verdict, Verdict::Unknown);
}

TEST(Resample, ATranslatedMapLandsOnTheSameWorldCells)
{
  // A SLAM map whose origin is half a metre east of the floor plan's, as a
  // map grown from a robot's start would be.
  auto geometry = square(-1);
  Grid slam;
  slam.width = 4;
  slam.height = 4;
  slam.resolution = 0.1;
  slam.origin_x = 0.5;
  slam.origin_y = 0.3;
  slam.cells.assign(16, 0);
  slam.cells[0] = 100;   // the cell at (0.55, 0.35)

  const auto out = pass_through::resample(slam, geometry);
  std::size_t i;
  ASSERT_TRUE(out.index(0.55, 0.35, i));
  EXPECT_EQ(out.cells[i], 100);
  ASSERT_TRUE(out.index(0.85, 0.65, i));
  EXPECT_EQ(out.cells[i], 0);
  ASSERT_TRUE(out.index(0.15, 0.15, i));
  EXPECT_EQ(out.cells[i], -1) << "the SLAM map says nothing here";
}

TEST(Floorplan, ReadsTheTrinaryConventionAndTheRowOrder)
{
  const std::string stem = "/tmp/pass_through_test_floorplan";
  {
    std::ofstream pgm(stem + ".pgm", std::ios::binary);
    pgm << "P5\n2 2\n255\n";
    // Top row (north): occupied, unknown. Bottom row (south): free, free.
    const unsigned char raster[] = {0, 205, 254, 254};
    pgm.write(reinterpret_cast<const char *>(raster), 4);
    std::ofstream yaml(stem + ".yaml");
    yaml << "image: pass_through_test_floorplan.pgm\nmode: trinary\nresolution: 0.5\n"
         << "origin: [-1.0, 2.0, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n";
  }
  const auto g = pass_through::load_map_yaml(stem + ".yaml");
  EXPECT_EQ(g.width, 2u);
  EXPECT_DOUBLE_EQ(g.origin_x, -1.0);
  EXPECT_DOUBLE_EQ(g.origin_y, 2.0);
  EXPECT_EQ(g.cells[0], 0);     // south-west
  EXPECT_EQ(g.cells[2], 100);   // north-west
  EXPECT_EQ(g.cells[3], -1);    // north-east
  std::remove((stem + ".pgm").c_str());
  std::remove((stem + ".yaml").c_str());
}
