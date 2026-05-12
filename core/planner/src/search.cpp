#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <iostream>
#include <algorithm>
#include <climits>

#include "search.hpp"
#include "product_update.hpp"
#include "bisimulation.hpp"

namespace gbfs {

struct Node {
    EpistemicState state;
    std::vector<std::string> plan;
    float h;
    bool operator>(const Node& o) const { return h > o.h; }
};

std::optional<SearchResult> search(const PlanningTask& task,
                                   const Heuristic& h,
                                   size_t max_nodes) {
    SearchResult result;

    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open;
    std::unordered_map<size_t, std::vector<EpistemicState>> closed;

    EpistemicState init = bisim_contract(task.init);

    if (init.satisfies(*task.goal)) {
        result.plan = {};
        return result;
    }

    open.push({std::move(init), {}, h(init, task)});

    while (!open.empty()) {
        Node node = std::move(const_cast<Node&>(open.top()));
        open.pop();

        result.nodes_expanded++;
        if (max_nodes > 0 && result.nodes_expanded > max_nodes) {
            std::cerr << "[gbfs] Node limit reached (" << max_nodes << ").\n";
            return std::nullopt;
        }

        size_t hsh = node.state.hash();
        {
            auto& bucket = closed[hsh];
            bool already_seen = std::any_of(bucket.begin(), bucket.end(),
                [&](const EpistemicState& s){ return s == node.state; });
            if (already_seen) continue;
            bucket.push_back(node.state);
        }

        for (auto& action : task.actions) {
            if (!action.applicable(node.state)) continue;

            auto maybe_next = product_update(node.state, action);
            if (!maybe_next) continue;

            EpistemicState next = bisim_contract(std::move(*maybe_next));
            result.nodes_generated++;

            std::vector<std::string> new_plan = node.plan;
            new_plan.push_back(action.name);

            if (next.satisfies(*task.goal)) {
                result.plan = std::move(new_plan);
                std::cerr << "[gbfs] Solution found! Length=" << result.plan.size()
                          << "  Expanded=" << result.nodes_expanded
                          << "  Generated=" << result.nodes_generated << "\n";
                return result;
            }

            float hval = h(next, task);
            size_t nhsh = next.hash();
            auto it = closed.find(nhsh);
            bool seen = it != closed.end() &&
                std::any_of(it->second.begin(), it->second.end(),
                    [&](const EpistemicState& s){ return s == next; });
            if (!seen)
                open.push({std::move(next), std::move(new_plan), hval});
        }
    }

    std::cerr << "[gbfs] Search exhausted  no solution.\n";
    return std::nullopt;
}

} // namespace gbfs

namespace aostar {

struct Stats {
    size_t nodes_expanded{0};
    size_t nodes_generated{0};
};

// Memo table key: state hash + depth remaining.
// Avoids re-expanding the same state at the same depth across branches.
using MemoKey = std::pair<size_t, size_t>;
struct MemoKeyHash {
    size_t operator()(const MemoKey& k) const {
        return k.first ^ (k.second * 0x9e3779b97f4a7c15ULL);
    }
};
using MemoTable = std::unordered_map<MemoKey, bool, MemoKeyHash>;

static std::vector<const Action*>
rank_actions(const EpistemicState& s,
             const PlanningTask& task,
             const Heuristic& h) {
    std::vector<std::pair<float, const Action*>> ranked;
    for (auto& a : task.actions) {
        if (!a.applicable(s)) continue;
        auto maybe = product_update(s, a);
        if (!maybe) continue;
        // no bisim here  just estimate cost from raw successor
        ranked.emplace_back(h(*maybe, task), &a);
    }
    std::sort(ranked.begin(), ranked.end(),
              [](auto& x, auto& y){ return x.first < y.first; });
    std::vector<const Action*> out;
    out.reserve(ranked.size());
    for (auto& [_, a] : ranked) out.push_back(a);
    return out;
}

static std::optional<std::shared_ptr<PlanNode>>
and_or_dfs(const EpistemicState& s,
           size_t depth,
           const PlanningTask& task,
           const Heuristic& h,
           Stats& stats,
           MemoTable& memo) {

    stats.nodes_expanded++;

    if (s.satisfies(*task.goal))
        return std::shared_ptr<PlanNode>{nullptr};

    if (depth == 0)
        return std::nullopt;

    MemoKey key{s.hash(), depth};
    auto mit = memo.find(key);
    if (mit != memo.end()) {
        // known failure at this (state, depth) skip
        if (!mit->second) return std::nullopt;
    }

    auto candidates = rank_actions(s, task, h);

    for (const Action* action : candidates) {
        auto branches = product_update_split(s, *action);
        if (branches.empty()) continue;

        stats.nodes_generated += branches.size();

        auto node = std::make_shared<PlanNode>();
        node->action = action->name;
        bool all_ok = true;

        for (auto& [eid, branch_state] : branches) {
            // bisim only at branch boundaries, not on every recursive step
            EpistemicState contracted = bisim_contract(branch_state);

            auto child = and_or_dfs(contracted, depth - 1, task, h, stats, memo);
            if (!child) {
                all_ok = false;
                break;
            }
            node->branches.emplace_back(eid, *child);
        }

        if (all_ok) {
            memo[key] = true;
            return node;
        }
    }

    memo[key] = false;
    return std::nullopt;
}

std::optional<ConditionalSearchResult>
search(const PlanningTask& task,
       const Heuristic& h,
       size_t max_depth) {

    EpistemicState init = bisim_contract(task.init);
    Stats stats;

    size_t depth_limit = (max_depth == 0) ? SIZE_MAX : max_depth;

    for (size_t depth = 0; depth <= depth_limit; depth++) {
        std::cerr << "[aostar] Trying depth " << depth << "\n";
        MemoTable memo;

        auto result = and_or_dfs(init, depth, task, h, stats, memo);

        if (result) {
            ConditionalSearchResult out;
            out.plan_tree       = *result;
            out.nodes_expanded  = stats.nodes_expanded;
            out.nodes_generated = stats.nodes_generated;
            std::cerr << "[aostar] Solution found at depth " << depth
                      << "  Expanded=" << stats.nodes_expanded
                      << "  Generated=" << stats.nodes_generated << "\n";
            return out;
        }
    }

    std::cerr << "[aostar] No solution within depth " << depth_limit << ".\n";
    return std::nullopt;
}

} // namespace aostar

namespace ehc {

// Enforced Hill Climbing
// At each step greedily pick any successor that strictly improves h.
// When no improving successor exists, run a BFS from the current state
// to find the nearest state with a better h value (the "escape" step).
// This makes EHC complete on solvable problems at the cost of the BFS plateau escape.
// Falls back to null if the BFS exhausts the reachable space.

std::optional<SearchResult> search(const PlanningTask& task,
                                   const Heuristic& h,
                                   size_t max_nodes) {
    SearchResult result;

    EpistemicState cur = bisim_contract(task.init);

    if (cur.satisfies(*task.goal)) {
        result.plan = {};
        return result;
    }

    std::vector<std::string> plan;
    std::unordered_map<size_t, std::vector<EpistemicState>> visited;

    float cur_h = h(cur, task);

    while (true) {
        if (max_nodes > 0 && result.nodes_expanded > max_nodes) {
            std::cerr << "[ehc] Node limit reached (" << max_nodes << ").\n";
            return std::nullopt;
        }

        //  greedy descent/ find any successor strictly better than cur_h 
        bool improved = false;
        for (auto& action : task.actions) {
            if (!action.applicable(cur)) continue;

            auto maybe_next = product_update(cur, action);
            if (!maybe_next) continue;

            EpistemicState next = bisim_contract(std::move(*maybe_next));
            result.nodes_generated++;

            if (next.satisfies(*task.goal)) {
                plan.push_back(action.name);
                result.plan = std::move(plan);
                result.nodes_expanded++;
                std::cerr << "[ehc] Solution found! Length=" << result.plan.size()
                          << "  Expanded=" << result.nodes_expanded
                          << "  Generated=" << result.nodes_generated << "\n";
                return result;
            }

            float nh = h(next, task);
            if (nh < cur_h) {
                cur   = std::move(next);
                cur_h = nh;
                plan.push_back(action.name);
                improved = true;
                result.nodes_expanded++;
                break; // it takes the first improvement (enforced hill climbing)
            }
        }

        if (improved) continue;

        //  plateau escape: BFS to nearest state with h < cur_h 
        std::cerr << "[ehc] Plateau at h=" << cur_h << "  BFS escape...\n";

        struct BFSNode {
            EpistemicState state;
            std::vector<std::string> suffix;
        };

        std::queue<BFSNode> bfs;
        std::unordered_map<size_t, std::vector<EpistemicState>> bfs_visited;

        bfs.push({cur, {}});
        size_t hsh0 = cur.hash();
        bfs_visited[hsh0].push_back(cur);

        bool escaped = false;

        while (!bfs.empty()) {
            auto node = std::move(bfs.front());
            bfs.pop();

            result.nodes_expanded++;
            if (max_nodes > 0 && result.nodes_expanded > max_nodes) {
                std::cerr << "[ehc] Node limit reached during BFS escape.\n";
                return std::nullopt;
            }

            for (auto& action : task.actions) {
                if (!action.applicable(node.state)) continue;

                auto maybe_next = product_update(node.state, action);
                if (!maybe_next) continue;

                EpistemicState next = bisim_contract(std::move(*maybe_next));
                result.nodes_generated++;

                if (next.satisfies(*task.goal)) {
                    for (auto& a : node.suffix) plan.push_back(a);
                    plan.push_back(action.name);
                    result.plan = std::move(plan);
                    std::cerr << "[ehc] Solution found during BFS escape! Length="
                              << result.plan.size()
                              << "  Expanded=" << result.nodes_expanded
                              << "  Generated=" << result.nodes_generated << "\n";
                    return result;
                }

                float nh = h(next, task);

                // Found escape: resume greedy descent from here
                if (nh < cur_h) {
                    for (auto& a : node.suffix) plan.push_back(a);
                    plan.push_back(action.name);
                    cur   = std::move(next);
                    cur_h = nh;
                    escaped = true;
                    goto escape_done;
                }

                // Add to BFS frontier if not yet visited
                size_t nhsh = next.hash();
                auto it = bfs_visited.find(nhsh);
                bool seen = it != bfs_visited.end() &&
                    std::any_of(it->second.begin(), it->second.end(),
                        [&](const EpistemicState& s){ return s == next; });
                if (!seen) {
                    bfs_visited[nhsh].push_back(next);
                    std::vector<std::string> new_suffix = node.suffix;
                    new_suffix.push_back(action.name);
                    bfs.push({std::move(next), std::move(new_suffix)});
                }
            }
        }
        escape_done:

        if (!escaped) {
            std::cerr << "[ehc] BFS escape exhausted  no solution.\n";
            return std::nullopt;
        }
    }
}

} // namespace ehc