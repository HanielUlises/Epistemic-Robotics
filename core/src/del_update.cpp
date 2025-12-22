#include "epistemic/del_update.hpp"
#include "epistemic/query.hpp"

namespace epistemic {

BeliefState product_update(
  const BeliefState& belief,
  const EventModel& event_model
) {
  BeliefState updated;

  // Create new worlds (w,e)
  for (WorldId w_id : belief.designated) {
    for (const Event& e : event_model.events) {
      if (holds(belief, w_id, e.precondition)) {
        World new_world = *std::find_if(
          belief.model.worlds.begin(),
          belief.model.worlds.end(),
          [&](const World& w) { return w.id == w_id; }
        );

        new_world.id =
          (static_cast<WorldId>(w_id) << 32) |
          static_cast<WorldId>(e.id);

        updated.model.worlds.push_back(new_world);
        updated.designated.push_back(new_world.id);
      }
    }
  }

  // Update accessibility
  for (const auto& [agent, rel] : belief.model.accessibility) {
    for (const auto& [w1, w2] : rel) {
      for (const Event& e1 : event_model.events) {
        for (const Event& e2 : event_model.events) {

          if (!event_model.accessible(agent, e1.id, e2.id))
            continue;

          if (!holds(belief, w1, e1.precondition)) continue;
          if (!holds(belief, w2, e2.precondition)) continue;

          WorldId new_w1 =
            (static_cast<WorldId>(w1) << 32) |
            static_cast<WorldId>(e1.id);

          WorldId new_w2 =
            (static_cast<WorldId>(w2) << 32) |
            static_cast<WorldId>(e2.id);

          updated.model.accessibility[agent].push_back(
            {new_w1, new_w2}
          );
        }
      }
    }
  }

  return updated;
}

} // namespace epistemic
