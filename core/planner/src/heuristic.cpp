#include "heuristic.hpp"

// h1: world count
// Fewer designated worlds -> less uncertainty -> closer to goal.
float WorldCountHeuristic::operator()(const EpistemicState& s,
                                       const PlanningTask&) const {
    return static_cast<float>(s.designated.size());
}

// h2: unsatisfied goal conjuncts
// Counts how many top-level conjuncts of the goal are not yet true.
// Falls back to h1 if goal is not a conjunction.
static float count_unsatisfied(const EpistemicState& s, const Formula& f) {
    if (f.kind == FormulaKind::And) {
        float unsat = 0.0f;
        for (auto& c : f.children)
            if (!s.satisfies(*c)) unsat += 1.0f;
        return unsat;
    }
    // Not a conjunction: 0 if satisfied, 1 if not
    return s.satisfies(f) ? 0.0f : 1.0f;
}

float UnsatisfiedGoalHeuristic::operator()(const EpistemicState& s,
                                            const PlanningTask& task) const {
    return count_unsatisfied(s, *task.goal);
}
