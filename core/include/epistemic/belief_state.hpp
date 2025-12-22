#pragma once
#include <vector>
#include "world.hpp"

namespace epistemic{

struct BeliefState {
  std::vector<World> worlds;

  bool empty() const;
};

} // namespace epistemic
