#pragma once

#include <unordered_map>
#include <vector>

#include "agent.hpp"
#include "formula.hpp"

namespace epistemic {

/**
 * A single epistemic event.
 */
struct Event {
  std::size_t id;
  Formula precondition;
};

/**
 * Event model with agent observability.
 */
struct EventModel {
  std::vector<Event> events;

  // R^E_a ⊆ Event × Event
  std::unordered_map<
    Agent,
    std::vector<std::pair<std::size_t, std::size_t>>
  > accessibility;

  bool accessible(
    Agent a,
    std::size_t e1,
    std::size_t e2
  ) const {
    auto it = accessibility.find(a);
    if (it == accessibility.end()) return false;

    for (const auto& [from, to] : it->second) {
      if (from == e1 && to == e2) return true;
    }
    return false;
  }
};

} // namespace epistemic
