#pragma once
#include "types.hpp"
#include "formula.hpp"

// One event inside an action's event model 
struct Event {
    EventIdx id;
    std::string name;

    FormulaPtr precondition;   // must hold in world for (world,event) to exist

    // postconditions: atom -> formula that gives new truth value
    // if atom not in map -> unchanged
    std::unordered_map<AtomIdx, FormulaPtr> post_true;   // atom becomes true if formula holds
    std::unordered_map<AtomIdx, FormulaPtr> post_false;  // atom becomes false if formula holds

    bool is_nil{false};        // trivial nil event (no effects)
};

// Abstract epistemic action = event model + observability
struct Action {
    std::string name;

    std::vector<Event> events;
    std::unordered_set<EventIdx> designated_events;   // E_d

    // event_accessibility[agent_idx][event_id] = set of reachable event ids
    std::vector<std::vector<std::unordered_set<EventIdx>>> event_accessibility;

    // observability conditions per agent:
    // obs_conditions[agent_idx] = formula that must hold for agent
    // to be "Fully" observant (see action type library semantics).
    // If formula holds -> agent sees real event; else -> sees nil event.
    std::vector<FormulaPtr> obs_conditions;

    size_t num_agents{0};

    //  Applicability: action is applicable in state s
    // iff ∃ designated event e such that pre(e) holds in some designated world
    bool applicable(const EpistemicState& s) const;
};
