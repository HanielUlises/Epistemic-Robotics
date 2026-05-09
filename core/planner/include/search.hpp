#pragma once
#include "task.hpp"
#include "heuristic.hpp"
#include <optional>
#include <vector>
#include <string>
#include <memory>

// Linear-plan search (GBFS)
struct SearchResult {
    std::vector<std::string> plan;
    size_t nodes_expanded{0};
    size_t nodes_generated{0};
};

namespace gbfs {

// Greedy Best-First Search.
// Returns the plan (action name sequence) on success, or nullopt if no
// plan exists within the given limits.
//
// max_nodes: expansion limit (0 = unlimited)
std::optional<SearchResult> search(const PlanningTask& task,
                                   const Heuristic& h,
                                   size_t max_nodes = 0);

} // namespace gbfs

// Conditional plan (AND-OR search)

// A node in a conditional plan tree.
// - action: the action to execute at this point
// - branches: one entry per sensing outcome (EventIdx tags which event fired).
//   For ontic actions there is exactly one branch with EventIdx = the single
//   designated event. For sensing actions there is one branch per designated
//   event whose precondition was satisfiable.
// - A null PlanNode pointer in a branch means that branch is already at goal.
struct PlanNode {
    std::string  action;
    std::vector<std::pair<EventIdx, std::shared_ptr<PlanNode>>> branches;
};

struct ConditionalSearchResult {
    std::shared_ptr<PlanNode> plan_tree;   // null = already at goal
    size_t nodes_expanded{0};
    size_t nodes_generated{0};
};

namespace aostar {

// Iterative-deepening AND-OR search.
// Returns a conditional plan tree on success, or nullopt if no plan exists
// within the given depth limit.
//
// max_depth: depth limit (0 = unlimited, use with care)
std::optional<ConditionalSearchResult>
search(const PlanningTask& task,
       const Heuristic& h,
       size_t max_depth = 30);

} // namespace aostar

// Enforced Hill Climbing.
// Greedily follows any h-improving successor. When stuck on a plateau,
// runs a BFS to escape to the nearest state with strictly lower h.
// Complete on solvable problems. Faster than GBFS on well-guided domains.
//
// max_nodes: expansion limit across both greedy and BFS phases (0 = unlimited)
namespace ehc {
std::optional<SearchResult> search(const PlanningTask& task,
                                   const Heuristic& h,
                                   size_t max_nodes = 0);
} // namespace ehc