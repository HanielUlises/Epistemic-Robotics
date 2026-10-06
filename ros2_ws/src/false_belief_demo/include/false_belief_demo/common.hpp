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

#ifndef FALSE_BELIEF_DEMO__COMMON_HPP_
#define FALSE_BELIEF_DEMO__COMMON_HPP_

#include <cmath>
#include <string>
#include <utility>
#include <vector>

#include "pass_through_demo/grid.hpp"

namespace false_belief
{

/// The value after `flag` on the command line, or `fallback`.
inline std::string argument(
  int argc, char ** argv, const std::string & flag, const std::string & fallback)
{
  for (int i = 1; i + 1 < argc; ++i) {
    if (flag == argv[i]) {return argv[i + 1];}
  }
  return fallback;
}

/// A point from a flat list of pairs, by index.
inline std::pair<double, double> pair_at(const std::vector<double> & flat, std::size_t i)
{
  return {flat.at(2 * i), flat.at(2 * i + 1)};
}

/// Whether the segment from a to b crosses no occupied cell of the floor plan
/// within `skip` metres of b. The tolerance at b is for the thing being looked
/// at, which is itself drawn as occupied.
///
/// This is the observability condition of the relocation read off the floor:
/// the domain says the picker at its dock does not see the bays, and the
/// floor-plan check and the performers check that from where it stands it
/// does not.
inline bool line_of_sight(
  const pass_through::Grid & plan, double ax, double ay, double bx, double by, double skip = 0.2)
{
  const double length = std::hypot(bx - ax, by - ay);
  const int steps = static_cast<int>(length / (plan.resolution / 3.0)) + 1;
  for (int k = 0; k <= steps; ++k) {
    const double t = static_cast<double>(k) / steps;
    const double x = ax + t * (bx - ax);
    const double y = ay + t * (by - ay);
    if (std::hypot(bx - x, by - y) <= skip) {
      break;
    }
    std::size_t i = 0;
    if (!plan.index(x, y, i)) {
      return false;
    }
    if (plan.cells[i] >= 65) {
      return false;
    }
  }
  return true;
}

}  // namespace false_belief

#endif  // FALSE_BELIEF_DEMO__COMMON_HPP_
