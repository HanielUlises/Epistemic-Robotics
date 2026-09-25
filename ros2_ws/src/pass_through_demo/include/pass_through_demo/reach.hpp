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

#ifndef PASS_THROUGH_DEMO__REACH_HPP_
#define PASS_THROUGH_DEMO__REACH_HPP_

#include <cstddef>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include "mu_path_planner/mu_calculus.hpp"
#include "pass_through_demo/grid.hpp"

namespace pass_through
{

// ---------------------------------------------------------------------------
// Where an agent may drive, as the extension of a formula
//
// The safe set of the route formula is not "the free cells". It is
//
//     Safe_i = [[ free  v  OR_t ( t  ^  K_i open_t ) ]]
//
// evaluated at every (cell, world) of the current pointed model: a cell is safe
// for agent i when i's map shows it free, or when it lies in a bay t and i
// knows that t is open. The second disjunct is what the knowledge precondition
// of `cross` looks like to the fixed point. A bay the carrier knows to be open
// by elimination -- which no robot has looked into, and which is unknown in
// every map -- is inside Safe_carrier all the same; a bay it does not know
// about is not, whatever the world says.
//
// `free` reads the agent's map. Outside the bays that is the floor plan, which
// every agent holds; inside them it is the agent's knowledge map, its own SLAM
// fused with whatever it has been sent. The formula is evaluated by
// mu_path_planner, over the model the epistemic state publishes, with the
// bays as zones.
// ---------------------------------------------------------------------------

struct SafeSetInput
{
  const Grid * floorplan{nullptr};  ///< common knowledge; the bays unknown
  const Grid * knowledge{nullptr};  ///< the agent's own; may be null
  std::map<std::string, Box> bays;  ///< zone name -> interior
  std::string agent;
  std::string model_json;           ///< the epistemic state's model; may be empty
  double inflation{0.35};           ///< metres obstacles are grown by
  epistemic_slam::Thresholds thresholds{};
};

struct SafeSet
{
  bool ok{false};
  std::string error;

  mu_path_planner::OccupancyGraph graph;   ///< obstacle = not safe
  std::vector<char> safe;

  /// Bays whose cells the formula made safe although the map does not show
  /// them free: the ones the agent knows to be open without having seen it.
  std::vector<std::string> lifted;

  /// Bays the agent knows to be open, by the model, whether or not its map
  /// also shows them.
  std::vector<std::string> known_open;

  /// The formula, as text, for the log.
  std::string formula;
};

/// Build the safe set. Without a model the second disjunct is false
/// everywhere, which is the right reading of "nothing is known yet".
SafeSet build_safe_set(const SafeSetInput & input);

// ---------------------------------------------------------------------------
// The least fixed point, evaluated semi-naively
//
//     W = mu Z . goal  v  ( Safe ^ <move> Z )
//
// mu_path_planner::mu_reach computes this by Kleene iteration, recomputing the
// backward image of the whole of Z_k at every step. Over this floor that is
// some five hundred iterations of a set of a hundred thousand cells, and
// tens of seconds. The iteration is monotone, so only the cells that entered
// at step k can bring new cells in at step k + 1; propagating from those alone
// is the textbook semi-naive evaluation of the same fixed point, and costs one
// visit per cell. `layer[c]` is the k at which c entered Z_k, so the region is
// the same set and the iteration count is the same number; test_reach checks
// both against mu_reach.
// ---------------------------------------------------------------------------

struct Reach
{
  std::vector<std::int32_t> layer;   ///< -1 outside W; k >= 1 inside
  std::uint32_t iterations{0};       ///< Kleene steps to the fixed point
  std::size_t size{0};               ///< |W|

  bool contains(std::size_t cell) const
  {
    return cell < layer.size() && layer[cell] >= 0;
  }
};

Reach least_fixed_point(
  const mu_path_planner::OccupancyGraph & graph,
  const std::vector<char> & goal,
  const std::vector<char> & safe);

/// A path from `start` into the goal, descending the layers: each step goes to
/// a neighbour that entered Z one iteration earlier. Empty when start is not in
/// W.
std::vector<std::size_t> descend(
  const mu_path_planner::OccupancyGraph & graph, const Reach & reach, std::size_t start);

/// The nearest cell to `from` that is in `region`, within `reach` cells by grid
/// distance, or `from` itself when it is already there. Used to start a route
/// from a robot parked inside the inflation band of a shelf.
bool nearest_in(
  const Grid & geometry, const std::vector<char> & region, std::size_t from,
  std::size_t reach, std::size_t & out);

/// Drop the waypoints a straight segment through safe cells makes redundant.
std::vector<std::size_t> shortcut(
  const Grid & geometry, const std::vector<char> & safe, const std::vector<std::size_t> & path);

/// The cells of `mask` whose eight neighbours are all in it. A shortcut checked
/// against this keeps a cell off the edge of the safe set, which a follower
/// that cuts corners otherwise spends.
std::vector<char> erode(const Grid & geometry, const std::vector<char> & mask);

}  // namespace pass_through

#endif  // PASS_THROUGH_DEMO__REACH_HPP_
