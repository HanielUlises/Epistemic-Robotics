#pragma once
#include "state.hpp"
#include "action.hpp"

// Compute s ⊗ a  (DEL product update).
//
// Returns the updated epistemic state, or std::nullopt if the action
// is not applicable (no designated event's precondition holds in any
// designated world)
std::optional<EpistemicState> product_update(const EpistemicState& s,
                                              const Action& a);
