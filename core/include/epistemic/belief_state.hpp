#pragma once

#include <vector>
#include <cstddef>

#include "world.hpp"
#include "kripke_model.hpp"

namespace epistemic {


// A belief state = Kripke model + designated worlds.
struct BeliefState {
  KripkeModel model;

  // Current possible worlds
  std::vector<WorldId> designated;

  bool empty() const {
    return designated.empty();
  }

  std::size_t size() const {
    return designated.size();
  }
};

} // namespace epistemic
