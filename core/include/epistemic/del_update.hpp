#pragma once

#include <algorithm>

#include "belief_state.hpp"
#include "event_model.hpp"

namespace epistemic {
BeliefState product_update(
  const BeliefState& B,
  const EventModel& E
);

} // namespace epistemic