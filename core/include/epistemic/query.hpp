#pragma once
#include "belief_state.hpp"
#include "formula.hpp"

namespace epistemic {
    
bool holds_in_all(
  const BeliefState& B,
  const Formula& phi
);

} // namespace epistemic
