#include "heuristic.hpp"
#include <unordered_set>
#include <algorithm>

float WorldCountHeuristic::operator()(const EpistemicState& s,
                                       const PlanningTask&) const {
    return static_cast<float>(s.designated.size());
}

static float count_unsatisfied(const EpistemicState& s, const Formula& f) {
    if (f.kind == FormulaKind::And) {
        float unsat = 0.0f;
        for (auto& c : f.children)
            if (!s.satisfies(*c)) unsat += 1.0f;
        return unsat;
    }
    return s.satisfies(f) ? 0.0f : 1.0f;
}

float UnsatisfiedGoalHeuristic::operator()(const EpistemicState& s,
                                            const PlanningTask& task) const {
    return count_unsatisfied(s, *task.goal);
}

// Hard cap on how many accessible worlds we inspect per heuristic call.
// Prevents hang on large universal-relation models.
static constexpr size_t MAX_SAMPLE = 64;

static float epistemic_distance_for_conjunct(const EpistemicState& s,
                                              const Formula& f) {
    if (s.satisfies(f)) return 0.0f;

    if (f.kind == FormulaKind::Belief) {
        AgentIdx ag = f.agent;
        if (ag >= s.accessibility.size()) return 1.0f;
        const Formula& inner = *f.children[0];

        size_t counterexamples = 0;
        size_t sampled = 0;

        for (WorldIdx w : s.designated) {
            for (WorldIdx v : s.accessibility[ag][w]) {
                if (!s.holds_at(inner, v))
                    counterexamples++;
                sampled++;
                if (sampled >= MAX_SAMPLE) goto done;
            }
        }
        done:
        if (sampled == 0) return 0.0f;
        return static_cast<float>(counterexamples) /
               static_cast<float>(sampled);
    }

    // Kw: [i]φ ∨ [i]¬φ — take the closer branch
    if (f.kind == FormulaKind::Or && f.children.size() == 2) {
        float d0 = epistemic_distance_for_conjunct(s, *f.children[0]);
        float d1 = epistemic_distance_for_conjunct(s, *f.children[1]);
        return std::min(d0, d1);
    }

    // Non-belief atom or other formula: check directly
    return s.satisfies(f) ? 0.0f : 1.0f;
}

float EpistemicDistanceHeuristic::operator()(const EpistemicState& s,
                                              const PlanningTask& task) const {
    const Formula& goal = *task.goal;
    if (goal.kind == FormulaKind::And) {
        float total = 0.0f;
        for (auto& c : goal.children)
            total += epistemic_distance_for_conjunct(s, *c);
        return total;
    }
    return epistemic_distance_for_conjunct(s, goal);
}