#pragma once
#include "state.hpp"
#include "task.hpp"

// Heuristic interface
// Lower value = closer to goal (GBFS minimises h).
struct Heuristic {
    virtual ~Heuristic() = default;
    virtual float operator()(const EpistemicState& s,
                             const PlanningTask& task) const = 0;
};

// h1: number of designated worlds.
// Intuition: fewer worlds implies less uncertainty which means closer to a
//            single-world goal state.
struct WorldCountHeuristic : Heuristic {
    float operator()(const EpistemicState& s,
                     const PlanningTask& task) const override;
};

// h2: number of goal conjuncts not yet satisfied.
// Is verified to work well when the goal is a conjunction of simple beliefs.
struct UnsatisfiedGoalHeuristic : Heuristic {
    float operator()(const EpistemicState& s,
                     const PlanningTask& task) const override;
};
