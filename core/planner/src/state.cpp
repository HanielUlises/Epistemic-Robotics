#include "state.hpp"
#include <functional>
#include <iostream>
#include <cassert>
#include <queue>
#include <algorithm>

// holds_at: evaluate formula f at world w 
bool EpistemicState::holds_at(const Formula& f, WorldIdx w) const {
    switch (f.kind) {
    case FormulaKind::Top:
        return true;

    case FormulaKind::Bot:
        return false;

    case FormulaKind::Atom:
        return worlds[w].atoms.count(f.atom) > 0;

    case FormulaKind::Not:
        return !holds_at(*f.children[0], w);

    case FormulaKind::And:
        for (auto& c : f.children)
            if (!holds_at(*c, w)) return false;
        return true;

    case FormulaKind::Or:
        for (auto& c : f.children)
            if (holds_at(*c, w)) return true;
        return false;

    case FormulaKind::Belief: {
        // [i]φ : for all v accessible from w via R_i, φ holds at v
        AgentIdx ag = f.agent;
        assert(ag < accessibility.size());
        const RelRow& row = accessibility[ag][w];
        for (WorldIdx v : row)
            if (!holds_at(*f.children[0], v)) return false;
        return true;
    }

    case FormulaKind::Common: {
        // [C.G]φ : φ holds at all worlds reachable via union of R_i (i∈G)
        // Implemented as BFS over the union relation.
        std::unordered_set<WorldIdx> visited;
        std::queue<WorldIdx> frontier;
        frontier.push(w);
        visited.insert(w);

        while (!frontier.empty()) {
            WorldIdx cur = frontier.front(); frontier.pop();
            if (!holds_at(*f.children[0], cur)) return false;
            for (AgentIdx ag : f.group) {
                for (WorldIdx v : accessibility[ag][cur]) {
                    if (!visited.count(v)) {
                        visited.insert(v);
                        frontier.push(v);
                    }
                }
            }
        }
        return true;
    }

    case FormulaKind::Kw:
        // Shouldn't reach here —> make_kw expands to Or{Belief, Belief}
        assert(false);
        return false;
    }
    return false; // unreachable
}

// satisfies: holds at all designated worlds
bool EpistemicState::satisfies(const Formula& f) const {
    for (WorldIdx w : designated)
        if (!holds_at(f, w)) return false;
    return true;
}

// hash 
// We hash all worlds reachable from designated worlds (via any agent's relation)
// and their atom sets. This ensures belief-formula-relevant worlds are covered.
size_t EpistemicState::hash() const {
    size_t h = 0;

    // BFS to collect all worlds reachable from designated
    std::unordered_set<WorldIdx> reachable(designated.begin(), designated.end());
    std::queue<WorldIdx> frontier;
    for (WorldIdx w : designated) frontier.push(w);
    while (!frontier.empty()) {
        WorldIdx cur = frontier.front(); frontier.pop();
        for (AgentIdx ag = 0; ag < accessibility.size(); ag++) {
            for (WorldIdx v : accessibility[ag][cur]) {
                if (reachable.insert(v).second)
                    frontier.push(v);
            }
        }
    }

    std::vector<WorldIdx> sorted_reachable(reachable.begin(), reachable.end());
    std::sort(sorted_reachable.begin(), sorted_reachable.end());

    for (WorldIdx w : sorted_reachable) {
        h ^= std::hash<uint32_t>{}(w) + 0x9e3779b9 + (h << 6) + (h >> 2);
        std::vector<AtomIdx> atoms(worlds[w].atoms.begin(), worlds[w].atoms.end());
        std::sort(atoms.begin(), atoms.end());
        for (AtomIdx a : atoms)
            // World index in atom hash to prevent XOR cancellation
            // across worlds with identical atom sets
            h ^= std::hash<uint32_t>{}(a + w * 1031u) + 0x9e3779b9 + (h << 6) + (h >> 2);
    }
    return h;
}

bool EpistemicState::operator==(const EpistemicState& o) const {
    if (designated != o.designated) return false;
    for (WorldIdx w : designated) {
        if (worlds[w].atoms != o.worlds[w].atoms) return false;
    }
    return accessibility == o.accessibility;
}

void EpistemicState::print(const std::vector<std::string>& atom_names,
                            const std::vector<std::string>& agent_names) const {
    std::cout << "Worlds: " << worlds.size()
              << "  Designated: {";
    for (WorldIdx w : designated) std::cout << w << " ";
    std::cout << "}\n";

    for (auto& world : worlds) {
        std::cout << "  w" << world.id;
        if (designated.count(world.id)) std::cout << "*";
        std::cout << ": {";
        for (AtomIdx a : world.atoms) {
            if (a < atom_names.size()) std::cout << atom_names[a] << " ";
            else std::cout << a << " ";
        }
        std::cout << "}\n";
    }
    for (AgentIdx ag = 0; ag < accessibility.size(); ag++) {
        std::string aname = (ag < agent_names.size()) ? agent_names[ag] : std::to_string(ag);
        std::cout << "  R_" << aname << ": ";
        for (WorldIdx w = 0; w < accessibility[ag].size(); w++) {
            for (WorldIdx v : accessibility[ag][w])
                std::cout << w << "->" << v << " ";
        }
        std::cout << "\n";
    }
}