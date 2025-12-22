#pragma once

#include <unordered_map>
#include <vector>
#include <utility>

#include "agent.hpp"
#include "world.hpp"

namespace epistemic {

/**
 * Kripke model:
 *  - a set of worlds
 *  - accessibility relations per agent
 */
struct KripkeModel {
  std::vector<World> worlds;

  // R_a ⊆ World × World
  std::unordered_map<
    Agent,
    std::vector<std::pair<WorldId, WorldId>>
  > accessibility;

  
  // Check world w2 is accessible from w1 for agent a.
  bool accessible(Agent a, WorldId w1, WorldId w2) const {
    auto it = accessibility.find(a);
    if (it == accessibility.end()) return false;

    for (const auto& [from, to] : it->second) {
      if (from == w1 && to == w2) return true;
    }
    return false;
  }
};

} // namespace epistemic
