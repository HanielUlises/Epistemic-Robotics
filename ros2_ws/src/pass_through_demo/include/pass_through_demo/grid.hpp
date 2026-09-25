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

#ifndef PASS_THROUGH_DEMO__GRID_HPP_
#define PASS_THROUGH_DEMO__GRID_HPP_

#include <cstddef>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include "epistemic_slam/map_fusion.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace pass_through
{

/// An axis-aligned box in metres, in the world frame.
struct Box
{
  double min_x{0.0}, min_y{0.0}, max_x{0.0}, max_y{0.0};

  bool contains(double x, double y) const
  {
    return x >= min_x && x <= max_x && y >= min_y && y <= max_y;
  }
};

/// Named boxes, parsed from a flat parameter: [name order] and four numbers per
/// name. ROS parameters cannot carry a map, so the launch passes both lists.
std::map<std::string, Box> boxes_from(
  const std::vector<std::string> & names, const std::vector<double> & flat);

/// An occupancy grid in the world frame, in the convention nav_msgs uses:
/// -1 unknown, 0..100 the probability of occupancy, rows running south to
/// north.
///
/// Every grid in this package is on one geometry, the floor plan's. A robot's
/// own SLAM map is not, and is resampled onto it on arrival; after that a
/// cell index means the same place in every map a node holds, which is what
/// lets two robots' maps be fused and a region be read the same way in both.
struct Grid
{
  std::uint32_t width{0};
  std::uint32_t height{0};
  double resolution{0.1};
  double origin_x{0.0};
  double origin_y{0.0};
  std::vector<std::int8_t> cells;

  std::size_t size() const {return cells.size();}
  bool empty() const {return cells.empty();}

  /// The cell holding (x, y), when there is one.
  bool index(double x, double y, std::size_t & out) const;

  /// The centre of cell `i`.
  void centre(std::size_t i, double & x, double & y) const;

  /// Every cell whose centre lies in the box.
  std::vector<std::size_t> cells_in(const Box & box) const;

  /// Same width, height, resolution and origin.
  bool same_geometry(const Grid & other) const;
};

/// A grid of the same geometry, every cell set to `value`.
Grid blank_like(const Grid & geometry, std::int8_t value = -1);

/// Read a map_server map: the YAML and the PGM it names, in trinary mode.
/// Throws std::runtime_error with the reason when either cannot be read.
Grid load_map_yaml(const std::string & yaml_path);

Grid from_msg(const nav_msgs::msg::OccupancyGrid & msg);
nav_msgs::msg::OccupancyGrid to_msg(const Grid & grid, const std::string & frame);

/// Put `source` onto `geometry`: each target cell takes the value of the source
/// cell under its centre, and stays unknown where the source says nothing.
///
/// A translation by whole cells when the resolutions agree, which they do here,
/// and not a registration: no rotation and no interpolation. It is correct
/// because every robot's odometry, and so its SLAM map frame, is the world
/// frame; a fleet whose frames differed would need a registration first, and
/// this would silently produce a map that is wrong by the offset.
Grid resample(const Grid & source, const Grid & geometry);

/// What a region of a map says, under the quantifier plansys2_epistemic_
/// perception uses: occupied anywhere is blocked, observed free everywhere is
/// clear, and anything else is undecided. Occupied beats unobserved because no
/// further looking makes an obstacle go away; unobserved beats free because a
/// region is only clear when all of it is.
enum class Verdict
{
  Unknown,
  Clear,
  Blocked,
};

const char * to_string(Verdict verdict);

struct RegionReading
{
  Verdict verdict{Verdict::Unknown};
  std::size_t cells{0};
  std::size_t free{0};
  std::size_t occupied{0};
  std::size_t unknown{0};
};

/// Read a region of a map. The cell classes are epistemic_slam's, so the
/// threshold between seen-and-settled and seen-and-undecided is the one the
/// fusion uses and is chosen in one place.
RegionReading read_region(
  const Grid & grid, const Box & box,
  const epistemic_slam::Thresholds & thresholds = {});

}  // namespace pass_through

#endif  // PASS_THROUGH_DEMO__GRID_HPP_
