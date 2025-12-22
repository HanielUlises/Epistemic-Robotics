#include "epistemic/del_update.hpp"
#include "epistemic/query.hpp"

namespace epistemic {

/**
 * Product update (simplified):
 *
 * Keep worlds for which there exists
 * an event whose precondition holds.
 */
BeliefState product_update(
  const BeliefState& belief,
  const EventModel& event_model
) {
  BeliefState updated;

  for (const World& w : belief.worlds) {
    for (const Event& e : event_model.events) {
      if (holds(w, e.precondition)) {
        updated.worlds.push_back(w);
        break;
      }
    }
  }

  return updated;
}

} // namespace epistemic
