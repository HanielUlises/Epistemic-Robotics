#pragma once
#include "task.hpp"
#include "heuristic.hpp"
#include <optional>
#include <vector>
#include <string>

struct SearchResult {
    std::vector<std::string> plan;   // action names in order
    size_t nodes_expanded{0};
    size_t nodes_generated{0};
};

// Greedy Best-First Search.
// Returns the plan (action name sequence) on success, or nullopt if no
// plan exists within the given limits.
//
// max_nodes: expansion limit (0 = unlimited)
std::optional<SearchResult> gbfs(const PlanningTask& task,
                                  const Heuristic& h,
                                  size_t max_nodes = 0);
