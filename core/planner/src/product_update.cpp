#include "product_update.hpp"
#include <algorithm>

// DEL Product Update  s ⊗ a 
//
// New worlds  : W' = { (w,e) | w ∈ W, e ∈ E, M,w ⊨ pre(e) }
// New relation: R'_i = { ((w,e),(v,f)) | w R_i v  ∧  e R^E_i f }
// New valuation: V'(w,e)(p) = post(e,p) evaluated at w
//                             (unchanged if p not in postconditions)
// New designated: W'* = { (w,e) | w ∈ W*, e ∈ E_d }
//
std::optional<EpistemicState> product_update(const EpistemicState& s,
                                              const Action& a) {
    size_t na = s.num_agents;

    std::unordered_map<uint64_t, WorldIdx> pair_to_idx;
    EpistemicState result;
    result.num_agents = na;

    auto encode_pair = [](WorldIdx w, EventIdx e) -> uint64_t {
        return (static_cast<uint64_t>(w) << 32) | static_cast<uint64_t>(e);
    };

    for (auto& world : s.worlds) {
        for (auto& event : a.events) {
            // Check pre(event) holds at world in s
            if (!s.holds_at(*event.precondition, world.id))
                continue;

            WorldIdx new_id = static_cast<WorldIdx>(result.worlds.size());
            pair_to_idx[encode_pair(world.id, event.id)] = new_id;

            World new_world;
            new_world.id   = new_id;
            new_world.atoms = world.atoms;

            // Apply postconditions of the event
            for (auto& [atom, cond] : event.post_true) {
                if (s.holds_at(*cond, world.id))
                    new_world.atoms.insert(atom);
            }
            for (auto& [atom, cond] : event.post_false) {
                if (s.holds_at(*cond, world.id))
                    new_world.atoms.erase(atom);
            }

            result.worlds.push_back(std::move(new_world));
        }
    }

    // designated worlds 
    // (w,e) is designated iff w ∈ W* AND e ∈ E_d
    for (WorldIdx w : s.designated) {
        for (EventIdx e : a.designated_events) {
            auto key = encode_pair(w, e);
            if (pair_to_idx.count(key))
                result.designated.insert(pair_to_idx.at(key));
        }
    }

    // Action is not applicable if no designated (world,event) pair exists
    if (result.designated.empty()) return std::nullopt;

    // accessibility relations
    size_t nw_new = result.worlds.size();
    result.accessibility.resize(na, Relation(nw_new));

    for (AgentIdx ag = 0; ag < na; ag++) {
        for (auto& [pair_we, new_w] : pair_to_idx) {
            WorldIdx w = static_cast<WorldIdx>(pair_we >> 32);
            EventIdx e = static_cast<EventIdx>(pair_we & 0xFFFFFFFF);

            // All (v,f) such that w R_i v AND e R^E_i f
            const RelRow& world_row = s.accessibility[ag][w];
            const auto&   event_row = a.event_accessibility[ag][e];

            for (WorldIdx v : world_row) {
                for (EventIdx f : event_row) {
                    auto key2 = encode_pair(v, f);
                    if (pair_to_idx.count(key2))
                        result.accessibility[ag][new_w].insert(pair_to_idx.at(key2));
                }
            }
        }
    }

    return result;
}

// Sensing product update: one EpistemicState per designated event.
std::vector<std::pair<EventIdx, EpistemicState>>
product_update_split(const EpistemicState& s, const Action& a) {
    // Run the full product update first to get the shared world/relation structure.
    auto maybe_full = product_update(s, a);
    if (!maybe_full) return {};

    const EpistemicState& full = *maybe_full;

    // We need to map new world indices back to which event produced them.
    // Re-derive the (w,e)->new_id mapping inline.
    size_t na = s.num_agents;

    auto encode_pair = [](WorldIdx w, EventIdx e) -> uint64_t {
        return (static_cast<uint64_t>(w) << 32) | static_cast<uint64_t>(e);
    };

    std::unordered_map<uint64_t, WorldIdx> pair_to_idx;
    {
        WorldIdx new_id = 0;
        for (auto& world : s.worlds) {
            for (auto& event : a.events) {
                if (!s.holds_at(*event.precondition, world.id)) continue;
                pair_to_idx[encode_pair(world.id, event.id)] = new_id++;
            }
        }
    }

    // For each designated event, collect the designated worlds that came from it.
    std::vector<std::pair<EventIdx, EpistemicState>> results;

    for (EventIdx eid : a.designated_events) {
        std::unordered_set<WorldIdx> branch_designated;

        for (WorldIdx w : s.designated) {
            auto key = encode_pair(w, eid);
            auto it = pair_to_idx.find(key);
            if (it != pair_to_idx.end())
                branch_designated.insert(it->second);
        }

        if (branch_designated.empty()) continue;

        // Build a sub-state: same worlds and accessibility as full,
        // but only this event's designated worlds.
        EpistemicState branch;
        branch.num_agents    = full.num_agents;
        branch.worlds        = full.worlds;
        branch.accessibility = full.accessibility;
        branch.designated    = std::move(branch_designated);

        results.emplace_back(eid, std::move(branch));
    }

    return results;
}