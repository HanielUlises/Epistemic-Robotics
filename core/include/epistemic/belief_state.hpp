#pragma once
// core/include/epistemic/belief_state.hpp
//
// BeliefState — the complete epistemic state at one time-step.
//
// A BeliefState bundles:
//   - a KripkeModel  (the full set of possible worlds + accessibility)
//   - designated     (which worlds are currently epistemically live)
//
// It is the primary object passed to product_update and holds/holds_in_all.

#include <cstddef>
#include <vector>

#include "kripke_model.hpp"
#include "world.hpp"

namespace epistemic {

struct BeliefState {
    // Callers should construct
    // via BeliefState{KripkeModel{agents}} when agent set is known.
    BeliefState() : model(std::set<std::string>{}) {}

    explicit BeliefState(KripkeModel m)
        : model(std::move(m)) {}

    KripkeModel           model;
    std::vector<WorldId>  designated;  // Epistemically live worlds

    bool        empty() const { return designated.empty(); }
    std::size_t size()  const { return designated.size(); }

    // True iff w_id is in the designated set
    bool is_designated(WorldId w_id) const {
        for (auto id : designated) if (id == w_id) return true;
        return false;
    }
};

} // namespace epistemic
