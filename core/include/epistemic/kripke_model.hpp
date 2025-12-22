#pragma once
#include <unordered_map>
#include <vector>
#include "agent.hpp"
#include "world.hpp"

namespace epistemic{

struct KripkeModel {
  std::vector<World> worlds;
  std::unordered_map<Agent, std::vector<std::pair<int,int>>> accessibility;
};

} // namespace epistemic