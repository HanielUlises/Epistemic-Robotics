#include "mu_path_planner/mu_calculus.hpp"

#include <queue>
#include <unordered_map>
#include <algorithm>

namespace mu_path_planner {

// ---------------------------------------------------------------------------
// OccupancyGraph
// ---------------------------------------------------------------------------

void OccupancyGraph::build_adjacency() {
    const size_t n = static_cast<size_t>(width) * height;
    neighbours.assign(n, {});

    const int dr[] = {-1, 1, 0, 0};
    const int dc[] = { 0, 0,-1, 1};

    for (uint32_t r = 0; r < height; ++r) {
        for (uint32_t c = 0; c < width; ++c) {
            CellIdx idx = cell(r, c);
            if (obstacle[idx]) continue;
            for (int d = 0; d < 4; ++d) {
                int nr = static_cast<int>(r) + dr[d];
                int nc = static_cast<int>(c) + dc[d];
                if (!in_bounds(nr, nc)) continue;
                CellIdx nidx = cell(static_cast<uint32_t>(nr),
                                    static_cast<uint32_t>(nc));
                if (!obstacle[nidx])
                    neighbours[idx].push_back(nidx);
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Backward image under ⟨move⟩
//
// pre(⟨move⟩Z) = { v | ∃ u ∈ Z, edge v→u }
// Since the graph is undirected (4-connectivity), pre = post = neighbours.
// ---------------------------------------------------------------------------

static std::unordered_set<CellIdx> backward_image(
    const OccupancyGraph& graph,
    const std::unordered_set<CellIdx>& Z)
{
    std::unordered_set<CellIdx> pre;
    for (CellIdx u : Z) {
        for (CellIdx v : graph.neighbours[u])
            pre.insert(v);
    }
    return pre;
}

// ---------------------------------------------------------------------------
// mu_reach — least fixed point
//
// Z_{k+1} = goal_set ∪ { v ∈ safe | v ∈ pre(⟨move⟩Z_k) }
// ---------------------------------------------------------------------------

MuReachResult mu_reach(
    const OccupancyGraph& graph,
    const std::unordered_set<CellIdx>& goal_set,
    const std::unordered_set<CellIdx>& safe_set,
    CellIdx start)
{
    MuReachResult result;

    // Z_0 = ∅
    std::unordered_set<CellIdx> Z;

    while (true) {
        result.iterations++;

        // Z_{k+1} = goal_set ∪ (safe ∩ pre(⟨move⟩Z))
        std::unordered_set<CellIdx> Z_next = goal_set;
        std::unordered_set<CellIdx> pre = backward_image(graph, Z);

        for (CellIdx v : pre) {
            if (safe_set.count(v))
                Z_next.insert(v);
        }

        if (Z_next == Z) break;   // fixed point reached
        Z = std::move(Z_next);
    }

    result.winning_region = Z;

    // Path extraction: BFS from start through winning region only
    if (!Z.count(start)) {
        // start ∉ winning region — no solution
        return result;
    }

    // BFS to nearest goal cell, restricted to winning_region
    std::unordered_map<CellIdx, CellIdx> parent;
    std::queue<CellIdx> frontier;
    frontier.push(start);
    parent[start] = start;

    CellIdx reached_goal = UINT32_MAX;

    while (!frontier.empty() && reached_goal == UINT32_MAX) {
        CellIdx cur = frontier.front();
        frontier.pop();

        if (goal_set.count(cur)) {
            reached_goal = cur;
            break;
        }

        for (CellIdx nb : graph.neighbours[cur]) {
            if (Z.count(nb) && !parent.count(nb)) {
                parent[nb] = cur;
                frontier.push(nb);
            }
        }
    }

    if (reached_goal == UINT32_MAX) return result;  // shouldn't happen

    // Reconstruct path
    std::vector<CellIdx> path;
    for (CellIdx c = reached_goal; c != start; c = parent.at(c))
        path.push_back(c);
    path.push_back(start);
    std::reverse(path.begin(), path.end());
    result.path = std::move(path);

    return result;
}

// ---------------------------------------------------------------------------
// mu_reach_epistemic
//
// μZ.(K_i(at_goal) ∨ (safe ∧ (⟨move⟩Z ∨ ⟨sense⟩Z)))
//
// ⟨sense⟩Z: from a sensing cell s, executing sense may produce K_i(at_goal).
// We model this as: s ∈ sensing_cells ∧ s ∈ safe → s ∈ Z_{k+1} if at_goal
// is *potentially* true at s (captured by sensing_cells being pre-computed
// as the set of cells within sensor range of the goal zone).
//
// In TT-II this will be tightened to depend on the actual KD45/S5 model.
// ---------------------------------------------------------------------------

EpistemicMuReachResult mu_reach_epistemic(
    const OccupancyGraph& graph,
    const std::unordered_set<CellIdx>& ontic_goal_set,
    const std::unordered_set<CellIdx>& safe_set,
    const std::unordered_set<CellIdx>& sensing_cells,
    CellIdx start)
{
    EpistemicMuReachResult result;

    // The epistemic goal set = ontic goal cells where the agent can *know*
    // it is there.  For now: K_i(at_goal) holds at cells where at_goal
    // is true AND the agent has sensed it (i.e., cell is in sensing_cells
    // and ontic_goal_set).
    std::unordered_set<CellIdx> epistemic_goal;
    for (CellIdx c : ontic_goal_set)
        if (sensing_cells.count(c))
            epistemic_goal.insert(c);

    // Extend reachable set with sensing cells adjacent to goal
    std::unordered_set<CellIdx> extended_goal = epistemic_goal;
    for (CellIdx s : sensing_cells)
        if (safe_set.count(s))
            extended_goal.insert(s);

    // Run mu_reach with extended goal (sensing cell = virtual goal reached
    // by ⟨sense⟩ action; path extractor will mark them as waypoints)
    MuReachResult base = mu_reach(graph, extended_goal, safe_set, start);

    result.winning_region = base.winning_region;
    result.path           = base.path;
    result.iterations     = base.iterations;

    // Tag sensing waypoints in the path
    for (CellIdx c : base.path)
        if (sensing_cells.count(c) && !ontic_goal_set.count(c))
            result.sensing_waypoints.push_back(c);

    return result;
}

}  // namespace mu_path_planner
