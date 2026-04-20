#pragma once
#include "state.hpp"

// Compute the bisimulation contraction of s.
// Returns a smallest state bisimilar to s (same formula truth values).
// Call this after every product_update() to keep state sizes manageable.
EpistemicState bisim_contract(const EpistemicState& s);
