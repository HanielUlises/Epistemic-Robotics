#pragma once
#include "state.hpp"
#include "task.hpp"

struct Heuristic {
    virtual ~Heuristic() = default;
    virtual float operator()(const EpistemicState& s,
                             const PlanningTask& task) const = 0;
};

// h1: number of designated worlds.
struct WorldCountHeuristic : Heuristic {
    float operator()(const EpistemicState& s,
                     const PlanningTask& task) const override;
};

// h2: number of goal conjuncts not yet satisfied.
struct UnsatisfiedGoalHeuristic : Heuristic {
    float operator()(const EpistemicState& s,
                     const PlanningTask& task) const override;
};

// h3: epistemic distance.
// For each unsatisfied belief conjunct [i]φ in the goal, counts the number
// of accessible worlds (from designated worlds) where φ fails.
// Gives a real gradient toward resolving epistemic uncertainty — unlike ug
// which only sees 0 or 1 per conjunct, ed sees how far each belief is from
// being true across the accessibility relation.
// For non-belief conjuncts falls back to ug (0 or 1).
// Combined: sum over all unsatisfied goal conjuncts.
struct EpistemicDistanceHeuristic : Heuristic {
    float operator()(const EpistemicState& s,
                     const PlanningTask& task) const override;
};