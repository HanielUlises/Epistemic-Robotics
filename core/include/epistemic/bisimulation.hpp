#pragma once

// Bisimulation contraction 
// Paige-Tarjan partition refinement.
//
// After a product_update the model may contain worlds that are
// epistemically equivalent (same truth values, same accessible
// neighbourhood up to equivalence).  Merging them keeps the model
// minimal and stabilises the EpistemicStateHasher output.
//
// Algorithm overview:
//   1. Initial partition: one block per unique valuation signature.
//   2. Repeat: split each block B where some agent a has worlds
//      w1,w2 ∈ B with different accessibility profiles relative to the
//      current partition.
//   3. Fixpoint: return the quotient BeliefState.
//
// Complexity: O(|W| · |A| · log|W|)  worlds up to ~10k
// worlds on a robot CPU.

#include <vector>

#include "belief_state.hpp"

namespace epistemic {

// Return a minimal BeliefState bisimilar to b.
// Designated worlds are mapped to their equivalence-class representatives.
BeliefState bisim_contraction(const BeliefState& b);

// True iff w1 and w2 are bisimilar in b.
bool bisimilar(const BeliefState& b, WorldId w1, WorldId w2);

// Return all worlds in the same bisimulation class as w.
std::vector<WorldId> bisim_class(const BeliefState& b, WorldId w);

} // namespace epistemic
