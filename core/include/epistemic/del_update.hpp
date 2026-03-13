#pragma once
// core/include/epistemic/del_update.hpp
//
// DEL product update and incremental variants.
//
// product_update(B, E)  — standard Baltag-Moss-Solecki product update.
//   Complexity: O(|designated|·|E|) world creation
//               O(|designated|²·|E|²) accessibility rebuild (designated-only)
//
// All accessibility work is restricted to the new designated set, giving
// a much tighter bound than the naive |W|²·|E|² over the full model.

#include "belief_state.hpp"
#include "event_model.hpp"

namespace epistemic {

// Standard product update always correct, uses designated-only inner loops.
BeliefState product_update(const BeliefState& B, const EventModel& E);

// Convenience: applies product_update then trims the accessibility
// relations to only those worlds that appear in the new designated set.
// This keeps the model small without running full bisim contraction.
BeliefState product_update_trim(const BeliefState& B, const EventModel& E);

} // namespace epistemic
