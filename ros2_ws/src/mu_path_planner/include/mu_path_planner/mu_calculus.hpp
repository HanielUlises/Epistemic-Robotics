#pragma once

#include <cstdint>
#include <vector>
#include <unordered_set>
#include <functional>
#include <optional>

namespace mu_path_planner {

// ---------------------------------------------------------------------------
// Grid representation
//
// The occupancy map from SLAM is discretised into a flat graph.
// Each cell is a node; edges connect 4-connected free neighbours.
// This mirrors the LTS that mu-calculus model-checks over: states are cells,
// the ⟨move⟩ modality is the 4-connectivity relation.
// ---------------------------------------------------------------------------

using CellIdx = uint32_t;

struct OccupancyGraph {
    uint32_t width{0};
    uint32_t height{0};

    // obstacle[i] = true  ↔  cell i is occupied / unknown
    std::vector<bool> obstacle;

    // Precomputed adjacency: neighbours[i] = free cells adjacent to i
    std::vector<std::vector<CellIdx>> neighbours;

    // Build adjacency from the obstacle mask (call after setting width/height/obstacle)
    void build_adjacency();

    CellIdx cell(uint32_t row, uint32_t col) const { return row * width + col; }
    bool in_bounds(int r, int c) const {
        return r >= 0 && c >= 0 &&
               static_cast<uint32_t>(r) < height &&
               static_cast<uint32_t>(c) < width;
    }
};

// ---------------------------------------------------------------------------
// Mu-calculus least fixed-point: backward reachability
//
// Computes the set  S* = μZ. φ_goal ∨ (φ_safe ∧ pre(⟨move⟩Z))
// where pre(⟨move⟩Z) = { v | ∃ u ∈ Z, v → u }  (backward image under move).
//
// Iteration:
//   Z_0 = ∅
//   Z_{k+1} = goal_set ∪ { v ∈ safe | ∃ u ∈ Z_k, v→u }
//   until Z_{k+1} = Z_k  (least fixed point reached)
//
// Returns the set of cells from which the goal is reachable while respecting
// the safety constraint.  The returned set IS the winning region of the
// mu-formula; a path is extracted greedily from start through it.
// ---------------------------------------------------------------------------

struct MuReachResult {
    // Winning region: set of cells satisfying μZ.(goal ∨ (safe ∧ ⟨move⟩Z))
    std::unordered_set<CellIdx> winning_region;

    // Greedy path from start to any goal cell, extracted from winning_region.
    // Empty if start ∉ winning_region (no solution).
    std::vector<CellIdx> path;

    // Fixed-point iteration count (diagnostic)
    uint32_t iterations{0};
};

/**
 * mu_reach
 *
 * Computes the least fixed point of the path-planning mu-formula over
 * the given occupancy graph.
 *
 * @param graph     The discretised free-space graph from SLAM.
 * @param goal_set  Cells satisfying the goal proposition (at_zone_X, etc.).
 * @param safe_set  Cells satisfying the safety constraint (¬obstacle).
 *                  Defaults to all non-obstacle cells if empty.
 * @param start     Starting cell for path extraction.
 * @return          MuReachResult with winning region and extracted path.
 */
MuReachResult mu_reach(
    const OccupancyGraph& graph,
    const std::unordered_set<CellIdx>& goal_set,
    const std::unordered_set<CellIdx>& safe_set,
    CellIdx start);

// ---------------------------------------------------------------------------
// Epistemic lifting
//
// When the goal is epistemic (K_i φ_goal), the formula becomes:
//
//   μZ.(K_i(at_goal) ∨ (¬obstacle ∧ (⟨move⟩Z ∨ ⟨sense⟩Z)))
//
// Here ⟨sense⟩Z means: the cell has a sensing action that, when executed,
// would make K_i(at_goal) true.  We model this as a set of "sensing cells"
// where the agent can observe whether at_goal holds.
//
// The sensing cells are derived from the agent's uncertainty over the goal
// zone in the current Kripke model: the cells some world it cannot rule out
// counts as goal cells and another does not. That derivation is in
// epistemic_state.hpp; this file only takes the fixed point.
// ---------------------------------------------------------------------------

struct EpistemicMuReachResult : MuReachResult {
    // Cells where a sensing action is needed (agent must stop and observe)
    std::vector<CellIdx> sensing_waypoints;
};

EpistemicMuReachResult mu_reach_epistemic(
    const OccupancyGraph& graph,
    const std::unordered_set<CellIdx>& ontic_goal_set,
    const std::unordered_set<CellIdx>& safe_set,
    const std::unordered_set<CellIdx>& sensing_cells,
    CellIdx start);

}  // namespace mu_path_planner
