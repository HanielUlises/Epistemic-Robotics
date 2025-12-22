#pragma once

#include "belief_state.hpp"
#include "formula.hpp"

namespace epistemic {

/**
 * Semantic satisfaction relation:
 *  model, w ⊨ phi
 */
bool holds(
  const BeliefState& belief,
  WorldId w,
  const Formula& phi
);

/**
 * True iff phi holds in all designated worlds.
 */
bool holds_in_all(
  const BeliefState& belief,
  const Formula& phi
);

} // namespace epistemic
