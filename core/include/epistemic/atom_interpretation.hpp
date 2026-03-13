// Interpret physical (ground) atoms against a World.
//
// This is the bridge between propositional formula atoms and the robot's
// physical state (GridMap, Pose, goals).  The Kripke model calls this for
// atoms it cannot find in its own valuation table.
//
// Supported:
//   cell_free(x,y)       — cell is CellState::Free
//   cell_occupied(x,y)   — cell is CellState::Occupied
//   cell_unknown(x,y)    — cell is CellState::Unknown
//   agent_at(a,x,y)      — agent a's pose maps to grid cell (x,y)
//   goal_is(a,label)     — agent a has goal string `label`

#include "world.hpp"
#include "formula.hpp"

namespace epistemic {

bool interpret_atom(const World& world, const Atom& atom);

} // namespace epistemic
