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

#include "epistemic_slam/map_fusion.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <string>

namespace epistemic_slam
{

namespace
{

/// Distance from the halfway point. A cell read as 5 or as 95 says something
/// definite; a cell read as 48 barely says anything. When two robots disagree
/// this is what decides which reading survives into the merged map.
int confidence(std::int8_t value)
{
  return std::abs(static_cast<int>(value) - 50);
}

bool near(double a, double b, double tolerance = 1e-6)
{
  return std::fabs(a - b) <= tolerance;
}

/// Both grids must describe the same discretisation of the same area.
///
/// Aligning two maps that do not is a registration problem, and a wrong answer
/// there produces a merged map that is confidently incorrect: cells attributed
/// to the wrong place, and robots believing they know things about somewhere
/// they have never been. Refusing is the honest outcome. In simulation the
/// robots share a map frame, so this holds by construction.
bool same_geometry(
  const nav_msgs::msg::OccupancyGrid & a,
  const nav_msgs::msg::OccupancyGrid & b,
  std::string & why)
{
  if (a.info.width != b.info.width || a.info.height != b.info.height) {
    why = "grids differ in size: " +
      std::to_string(a.info.width) + "x" + std::to_string(a.info.height) + " and " +
      std::to_string(b.info.width) + "x" + std::to_string(b.info.height);
    return false;
  }
  if (!near(a.info.resolution, b.info.resolution)) {
    why = "grids differ in resolution: " +
      std::to_string(a.info.resolution) + " and " + std::to_string(b.info.resolution);
    return false;
  }
  if (!near(a.info.origin.position.x, b.info.origin.position.x) ||
    !near(a.info.origin.position.y, b.info.origin.position.y) ||
    !near(a.info.origin.position.z, b.info.origin.position.z) ||
    !near(a.info.origin.orientation.x, b.info.origin.orientation.x) ||
    !near(a.info.origin.orientation.y, b.info.origin.orientation.y) ||
    !near(a.info.origin.orientation.z, b.info.origin.orientation.z) ||
    !near(a.info.origin.orientation.w, b.info.origin.orientation.w))
  {
    why = "grids differ in origin; aligning them is a registration problem "
      "this does not attempt";
    return false;
  }

  const std::size_t expected =
    static_cast<std::size_t>(a.info.width) * static_cast<std::size_t>(a.info.height);
  if (a.data.size() != expected || b.data.size() != expected) {
    why = "grid data does not match its declared size";
    return false;
  }
  return true;
}

}  // namespace

CellClass classify(std::int8_t value, const Thresholds & thresholds)
{
  if (!is_known(value)) {
    return CellClass::Unknown;
  }
  if (value < thresholds.free_below) {
    return CellClass::Free;
  }
  if (value > thresholds.occupied_above) {
    return CellClass::Occupied;
  }
  // Seen, and still undecided. Reporting it either way would hand the
  // epistemic layer a certainty the sensor never produced.
  return CellClass::Unknown;
}

Coverage coverage(const nav_msgs::msg::OccupancyGrid & grid)
{
  Coverage observed(grid.data.size(), false);
  for (std::size_t i = 0; i < grid.data.size(); ++i) {
    observed[i] = is_known(grid.data[i]);
  }
  return observed;
}

double coverage_fraction(const nav_msgs::msg::OccupancyGrid & grid)
{
  if (grid.data.empty()) {
    return 0.0;
  }
  std::size_t known = 0;
  for (const auto value : grid.data) {
    if (is_known(value)) {
      ++known;
    }
  }
  return static_cast<double>(known) / static_cast<double>(grid.data.size());
}

Fusion fuse(
  const nav_msgs::msg::OccupancyGrid & a,
  const nav_msgs::msg::OccupancyGrid & b,
  const Thresholds & thresholds)
{
  Fusion result;

  if (!same_geometry(a, b, result.error)) {
    return result;
  }

  result.merged.header = a.header;
  result.merged.info = a.info;
  result.merged.data.resize(a.data.size());

  for (std::size_t i = 0; i < a.data.size(); ++i) {
    const std::int8_t va = a.data[i];
    const std::int8_t vb = b.data[i];
    const bool known_a = is_known(va);
    const bool known_b = is_known(vb);

    if (!known_a && !known_b) {
      result.merged.data[i] = -1;
      continue;
    }

    if (known_a && !known_b) {
      result.merged.data[i] = va;
      result.newly_known_to_b.push_back(i);
      continue;
    }

    if (!known_a && known_b) {
      result.merged.data[i] = vb;
      result.newly_known_to_a.push_back(i);
      continue;
    }

    // Both have seen it. Disagreeing about confidence is ordinary; disagreeing
    // about whether the cell is passable is not, and only the second is worth
    // reporting to anyone.
    const CellClass ca = classify(va, thresholds);
    const CellClass cb = classify(vb, thresholds);
    const bool contradiction =
      (ca == CellClass::Free && cb == CellClass::Occupied) ||
      (ca == CellClass::Occupied && cb == CellClass::Free);

    if (contradiction) {
      result.conflicts.push_back(Conflict{i, va, vb});
    }

    // The more confident reading wins, including when they contradict each
    // other: a merged map with a disputed cell someone can go and look at
    // again is more useful than one with a hole punched in it.
    result.merged.data[i] = confidence(vb) > confidence(va) ? vb : va;
  }

  result.ok = true;
  return result;
}

}  // namespace epistemic_slam
