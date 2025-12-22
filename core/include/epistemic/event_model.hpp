#pragma once

#include <vector>

#include "formula.hpp"

namespace epistemic {

// A single epistemic event
struct Event {
  Formula precondition;
};

// An event model represents uncertainty about which event occurred.
struct EventModel {
  std::vector<Event> events;
};

} // namespace epistemic
