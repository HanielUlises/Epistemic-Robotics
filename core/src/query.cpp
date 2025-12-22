#include "epistemic/query.hpp"
#include "epistemic/atom_interpretation.hpp"

#include <algorithm>

namespace epistemic {

static const World* find_world(
  const KripkeModel& model,
  WorldId id
) {
  for (const auto& w : model.worlds) {
    if (w.id == id) return &w;
  }
  return nullptr;
}

bool holds(
  const BeliefState& belief,
  WorldId w_id,
  const Formula& phi
) {
  const World* w = find_world(belief.model, w_id);
  if (!w) return false;

  return std::visit([&](auto&& arg) -> bool {

    using T = std::decay_t<decltype(arg)>;

    // Atomic proposition
    if constexpr (std::is_same_v<T, Atom>) {
        return interpret_atom(*w, arg);
    }

    // Negation
    else if constexpr (std::is_same_v<T, Not>) {
      return !holds(belief, w_id, *arg.phi);
    }

    // Conjunction
    else if constexpr (std::is_same_v<T, And>) {
      return holds(belief, w_id, *arg.left)
          && holds(belief, w_id, *arg.right);
    }

    // Knowledge operator
    else if constexpr (std::is_same_v<T, Knows>) {
      Agent a = arg.agent;

      for (WorldId w2_id : belief.designated) {
        if (belief.model.accessible(a, w_id, w2_id)) {
          if (!holds(belief, w2_id, *arg.phi)) {
            return false;
          }
        }
      }
      return true;
    }

    else {
      return false;
    }

  }, phi.value);
}

bool holds_in_all(
  const BeliefState& belief,
  const Formula& phi
) {
  for (WorldId w_id : belief.designated) {
    if (!holds(belief, w_id, phi)) {
      return false;
    }
  }
  return true;
}

} // namespace epistemic
