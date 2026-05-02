#include "action.hpp"
#include "state.hpp"

// An action is applicable in state s iff every designated event e ∈ E_d
// has its precondition satisfied in every designated world w ∈ W*.
// (Strong executability — standard DEL semantics.)
bool Action::applicable(const EpistemicState& s) const {
    if (designated_events.empty()) return false;
    for (EventIdx eid : designated_events) {
        if (eid >= events.size()) return false;
        const Event& ev = events[eid];
        for (WorldIdx w : s.designated) {
            if (!s.holds_at(*ev.precondition, w))
                return false;
        }
    }
    return true;
}

bool Action::applicable_weak(const EpistemicState& s) const {
    for (EventIdx eid : designated_events) {
        if (eid >= events.size()) return false;
        const Event& ev = events[eid];
        for (WorldIdx w : s.designated)
            if (s.holds_at(*ev.precondition, w))
                return true;
    }
    return false;
}