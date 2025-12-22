#pragma once

#include "world.hpp"
#include "formula.hpp"

namespace epistemic {

/**
 * Interpret an atomic proposition in a given world.
 *
 * e.g.
 *  - "cell_free(3,4)"
 *  - "agent_at(robot, x, y)"
 *  - "goal_reached(robot)"
 */
bool interpret_atom(
  const World& world,
  const Atom& atom
);

} // namespace epistemic
