#pragma once
#include <vector>
#include "formula.hpp"

namespace epistemic {
struct Event {
  Formula precondition;
};

struct EventModel {
  std::vector<Event> events;
};

} // namespace epistemic