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

#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "mu_path_planner/mu_calculus.hpp"

// ---------------------------------------------------------------------------
// The bridge between the Kripke snapshot on /epistemic/state and the sets the
// mu-calculus fixed point runs over.
//
// mu_calculus.hpp knows about cells and nothing else: a goal set, a safe set,
// a set of cells where sensing pays.  Where those three sets come from is the
// question answered here, and it is an epistemic question, not a geometric
// one.  Nothing in this file talks to ROS, so it is tested against snapshot
// text with no graph and no map.
//
// ---------------------------------------------------------------------------
// Snapshot format
//
// The Kripke part is plank's, so that a model produced by the planner can be
// forwarded unchanged.  The grounding part is this repository's: cells are not
// atoms, and something has to say which cells a zone name stands for.
//
// {
//   "worlds":     ["w0", "w1"],
//   "designated": ["w0"],
//   "relations":  { "r1": { "w0": ["w0","w1"], "w1": ["w0","w1"] } },
//   "labels":     { "w0": ["corridor_clear"], "w1": [] },
//   "agents":     { "1": { "name": "r1", "cell": 145 } },
//   "zones": {
//     "bay2": { "cells": [10, 11, 12] },
//     "depot": { "rect":   { "row": 4, "col": 2, "height": 3, "width": 3 } },
//     "dock":  { "bounds": { "min_x": 1.0, "min_y": 0.5,
//                            "max_x": 2.0, "max_y": 1.5 } },
//     "corridor": {
//       "worlds": { "w0": { "cells": [30, 31] }, "w1": { "cells": [] } }
//     }
//   }
// }
//
// An agent is located by "cell" (a flat index) or by "pose": {"x":..,"y":..}
// in map metres, which is resolved against the grid at parse time.  A zone
// given by "cells", "rect" or "bounds" has the same extent in every world; a
// zone given by "worlds" has the extent listed there and is empty in worlds
// not listed.  That difference is the whole point: an agent knows it has
// reached a zone exactly when every world it cannot rule out agrees that the
// cell it stands on belongs to the zone.
//
// ---------------------------------------------------------------------------
// Formulas
//
// Safety constraints arrive as plank formula JSON, the same shape plank emits
// for preconditions and goals:
//
//   "atom"                                        an atom, or "true" / "false"
//   { "connective": "not", "formula": F }
//   { "connective": "and" | "or", "formulas": [ F, ... ] }
//   { "connective": "imply", "formulas": [ F, G ] }
//   { "modality-name": "box", "modality-index": ["r1"], "formula": F }
//
// Modalities: box (K_i), diamond, Kw.box (knowing whether), C.box (common
// knowledge over the index), D.box (distributed knowledge over the index).
//
// Atoms are evaluated at a pair (cell, world).  A zone name is true at
// (c, w) when c belongs to that zone in w.  The atoms obstacle, free and
// unknown read the map rather than the model.  Anything else is an ordinary
// propositional atom and is true at (c, w) when w's label set contains it,
// which makes it independent of the cell.
// ---------------------------------------------------------------------------

namespace mu_path_planner {

// ---------------------------------------------------------------------------
// Formulas
// ---------------------------------------------------------------------------

struct Formula;
using FormulaPtr = std::shared_ptr<const Formula>;

struct Formula {
    enum class Kind { True, False, Atom, Not, And, Or, Imply, Modal };

    // Which modality a Kind::Modal node carries.
    enum class Modality { Box, Diamond, KnowWhether, CommonBox, DistributedBox };

    Kind kind{Kind::True};

    std::string atom;                   ///< Kind::Atom
    Modality modality{Modality::Box};   ///< Kind::Modal
    std::vector<std::string> index;     ///< Kind::Modal: the agents it ranges over
    std::vector<FormulaPtr> subs;       ///< operands, in order
};

/// Parses plank formula JSON.  Returns nullptr and sets @p error on malformed
/// input; an empty or whitespace-only text is malformed, since a caller that
/// means "no constraint" should not be asking.
FormulaPtr parse_formula(const std::string& json_text, std::string& error);

// ---------------------------------------------------------------------------
// The snapshot
// ---------------------------------------------------------------------------

/// How a cell reads once the occupancy thresholds are applied.  Unknown is not
/// free: a cell nobody observed is excluded from the fixed point rather than
/// driven through.
enum class CellState : uint8_t { Unknown, Free, Occupied };

/// Enough of the map to resolve metres into cells.
struct GridInfo {
    uint32_t width{0};
    uint32_t height{0};
    double resolution{0.05};
    double origin_x{0.0};
    double origin_y{0.0};

    std::optional<CellIdx> cell_at(double x, double y) const;
};

/// Where an agent is, as the snapshot reports it.
struct AgentInfo {
    std::string name;
    std::optional<CellIdx> cell;
};

/// The cells a zone covers, possibly world by world.
struct ZoneGrounding {
    /// Extent shared by every world, when the zone was given without "worlds".
    std::optional<std::unordered_set<CellIdx>> global;

    /// Extent per world, when it was.  A world absent from here has no cells.
    std::unordered_map<std::string, std::unordered_set<CellIdx>> per_world;

    const std::unordered_set<CellIdx>* cells_in(const std::string& world) const;
};

/// A pointed Kripke model plus the grounding that ties its zone names to
/// cells.  Parsed once per message on /epistemic/state.
struct EpistemicSnapshot {
    bool ok{false};
    std::string error;

    std::vector<std::string> worlds;
    std::vector<std::string> designated;

    /// relations[agent][world] = the worlds that agent cannot tell it from.
    /// An agent or a world missing from here is taken to be reflexive only,
    /// which is the S5 reading of a relation nobody constrained.
    std::unordered_map<
        std::string,
        std::unordered_map<std::string, std::vector<std::string>>> relations;

    /// labels[world] = the propositional atoms true there.
    std::unordered_map<std::string, std::unordered_set<std::string>> labels;

    std::unordered_map<uint32_t, AgentInfo> agents;
    std::unordered_map<std::string, ZoneGrounding> zones;

    /// The worlds agent @p agent cannot distinguish from @p world.
    std::vector<std::string> accessible(const std::string& agent,
                                        const std::string& world) const;

    /// The transitive closure of the union of the relations of @p group,
    /// starting at @p world: the worlds common knowledge quantifies over.
    std::vector<std::string> common_accessible(
        const std::vector<std::string>& group, const std::string& world) const;
};

/// Parses a snapshot.  @p grid is needed because poses and metric zone bounds
/// are resolved to cells here rather than left for the caller to redo.
EpistemicSnapshot parse_snapshot(const std::string& json_text,
                                 const GridInfo& grid);

// ---------------------------------------------------------------------------
// Evaluation
// ---------------------------------------------------------------------------

/// What an atom is evaluated against.  @p cell_state may be empty, in which
/// case obstacle and free fall back to the graph's obstacle mask and unknown
/// is nowhere true.
struct EvalContext {
    const EpistemicSnapshot* snapshot{nullptr};
    const OccupancyGraph* graph{nullptr};
    const std::vector<CellState>* cell_state{nullptr};
};

/// Truth of @p f at cell @p cell in world @p world.
bool holds(const Formula& f, CellIdx cell, const std::string& world,
           const EvalContext& ctx);

/// Truth of @p f at @p cell in every designated world, which is what it means
/// for a formula to hold in the situation as far as the agent can tell.
bool holds_at_designated(const Formula& f, CellIdx cell, const EvalContext& ctx);

// ---------------------------------------------------------------------------
// Resolving a query into the sets the fixed point runs over
// ---------------------------------------------------------------------------

/// A MuPathQuery, with the ROS types stripped off.
struct QuerySpec {
    uint32_t agent_id{0};
    std::string goal_zone;
    std::string safety_formula_json;    ///< empty means ¬obstacle
    bool require_epistemic_goal{false};

    /// How far a sensing action reaches, in cells, measured over the grid and
    /// not over free space: a sensor sees across a cell it has not resolved.
    /// One means the agent has to stand on or next to the cell it resolves.
    uint32_t sensor_range_cells{1};
};

/// The sets mu_reach and mu_reach_epistemic take, derived from the snapshot.
struct ResolvedQuery {
    bool ok{false};
    std::string error;

    CellIdx start{0};

    /// Cells in the goal zone in every designated world: where the agent
    /// would in fact be at the goal.
    std::unordered_set<CellIdx> goal;

    /// Cells satisfying the safety formula in every designated world, minus
    /// the cells that are not free.
    std::unordered_set<CellIdx> safe;

    /// Where sensing is worth spending, plus the goal cells the agent already
    /// knows about.  Empty unless the query asked for an epistemic goal.
    std::unordered_set<CellIdx> sensing;

    /// Goal cells the agent already knows to be goal cells: the ones where
    /// K_i(at_goal) holds without sensing anything.  A subset of goal.
    std::unordered_set<CellIdx> known_goal;

    /// Cells the agent cannot decide the goal zone about: in the zone in some
    /// world it cannot rule out, out of it in another.  These are what a
    /// sensing action is for, and they are what sensing dilates from.  They
    /// are not filtered to free cells, because the cell an agent cannot
    /// decide about is usually one it has not observed.
    std::unordered_set<CellIdx> disputed;
};

/// Turns a query and a snapshot into goal, safe and sensing sets.
///
/// Fails, rather than guessing, when the snapshot does not locate the agent or
/// does not ground the goal zone: a plan from a start the planner invented is
/// worse than no plan.
ResolvedQuery resolve_query(const EpistemicSnapshot& snapshot,
                            const OccupancyGraph& graph,
                            const std::vector<CellState>& cell_state,
                            const QuerySpec& spec);

}  // namespace mu_path_planner
