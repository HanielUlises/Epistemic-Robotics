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

#ifndef EPISTEMIC_SLAM__MAP_FUSION_HPP_
#define EPISTEMIC_SLAM__MAP_FUSION_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include "nav_msgs/msg/occupancy_grid.hpp"

namespace epistemic_slam
{

/// What a robot has actually observed, as a mask over the grid.
///
/// This is not the same question as which cells are free in the map a robot
/// holds. A robot that has been handed a merged map holds cells it never saw,
/// and the difference is the whole point: an agent knows what it observed, not
/// what it was told without being told who observed it.
using Coverage = std::vector<bool>;

/// The three states a cell can be in once thresholds are applied. An
/// occupancy grid stores a probability and the epistemic layer needs a
/// decision, so somewhere a threshold has to be chosen; it is chosen here and
/// nowhere else.
enum class CellClass
{
  Unknown,    ///< never observed, or observed inconclusively
  Free,
  Occupied,
};

/// Where the line falls between free, occupied and undecided.
///
/// The defaults match the convention nav2 and slam_toolbox use for a costmap
/// read back as an OccupancyGrid: below 25 is free enough to drive, above 65
/// is an obstacle, and the band between them is a cell that has been seen
/// without being settled. A cell in that band is Unknown on purpose: reporting
/// it as free would let a robot claim knowledge it does not have.
struct Thresholds
{
  std::int8_t free_below{25};
  std::int8_t occupied_above{65};
};

/// One cell where two robots disagree: one says free, the other says occupied.
///
/// Not every difference is a conflict. Disagreeing about how confident to be
/// is ordinary; disagreeing about whether a corridor is passable is not, and
/// only the second kind is reported. Thesis section 3.9.4.
struct Conflict
{
  std::size_t index{0};
  std::int8_t value_a{-1};
  std::int8_t value_b{-1};
};

/// The result of reconciling two maps when a link comes back.
struct Fusion
{
  bool ok{false};
  std::string error;              ///< why the maps could not be reconciled

  nav_msgs::msg::OccupancyGrid merged;

  /// Cells the other robot knew and this one did not. This is what each robot
  /// learns from the exchange, and therefore what the epistemic state has to
  /// be told about.
  std::vector<std::size_t> newly_known_to_a;
  std::vector<std::size_t> newly_known_to_b;

  /// Cells where the two maps contradict each other.
  std::vector<Conflict> conflicts;
};

/// True when the cell has been observed at all.
inline bool is_known(std::int8_t value) {return value >= 0;}

/// Classify one cell value against the thresholds.
CellClass classify(std::int8_t value, const Thresholds & thresholds = {});

/// The cells this robot has observed.
Coverage coverage(const nav_msgs::msg::OccupancyGrid & grid);

/// How much of the grid a robot has observed, as a fraction in [0, 1]. The
/// coverage metric OE4 asks for.
double coverage_fraction(const nav_msgs::msg::OccupancyGrid & grid);

/// Reconcile two maps of the same area.
///
/// Both grids must describe the same discretisation: same width, height,
/// resolution and origin. In simulation the robots share a map frame, so this
/// holds by construction; on hardware it would not, and aligning two maps is a
/// different problem that this deliberately refuses rather than approximates.
/// A mismatch returns ok = false with the reason.
///
/// Where only one robot has seen a cell, its value is taken. Where both have,
/// the more confident reading wins, meaning the one further from 50. Where
/// they contradict each other, the conflict is recorded and the more confident
/// reading still wins, because a merged map with a hole in it is worse than a
/// merged map with a disputed cell someone can go and look at again.
Fusion fuse(
  const nav_msgs::msg::OccupancyGrid & a,
  const nav_msgs::msg::OccupancyGrid & b,
  const Thresholds & thresholds = {});

}  // namespace epistemic_slam

#endif  // EPISTEMIC_SLAM__MAP_FUSION_HPP_
