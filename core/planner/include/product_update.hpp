#pragma once
#include "state.hpp"
#include "action.hpp"
#include <vector>

// Compute s ⊗ a  (DEL product update).
//
// Returns the updated epistemic state, or std::nullopt if the action
// is not applicable (no designated event's precondition holds in any
// designated world)
std::optional<EpistemicState> product_update(const EpistemicState& s,
                                              const Action& a);

// Sensing variant: returns one EpistemicState per designated event.
//
// For ontic actions (single designated event) this returns a vector of
// size 1, identical to what product_update returns.
// For sensing actions (multiple designated events) this partitions the
// product-update result by which designated event fired, producing one
// sub-state per event. Each sub-state shares the same world set and
// accessibility as the full product update but has a distinct designated
// set — the worlds (w,e) where e is exactly that one designated event.
//
// Returns an empty vector if the action is not applicable.
// Each element is tagged with the EventIdx that produced it.
std::vector<std::pair<EventIdx, EpistemicState>>
product_update_split(const EpistemicState& s, const Action& a);