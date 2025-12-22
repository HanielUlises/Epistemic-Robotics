#pragma once

#include <cstdint>
#include <unordered_map>
#include <string>
#include <vector>

#include "agent.hpp"

namespace epistemic {

using WorldId = std::uint64_t;

/**
 * Simple 2D pose for agents.
 * You can generalize later (SE(3), covariance, etc.).
 */
struct Pose {
  double x;
  double y;
  double theta;
};

/**
 * Occupancy state for a map cell.
 */
enum class CellState : std::uint8_t {
  Unknown,
  Free,
  Occupied
};

/**
 * Discrete grid map representation.
 * This is intentionally minimal and epistemic-friendly.
 */
struct GridMap {
  std::uint32_t width;
  std::uint32_t height;
  double resolution; // meters per cell

  std::vector<CellState> cells;

  CellState at(std::uint32_t x, std::uint32_t y) const {
    return cells[y * width + x];
  }
};

/**
 * A single epistemic world.
 *
 * This represents ONE hypothesis about:
 *  - the map
 *  - agent poses
 *  - agent goals
 */
struct World {
  WorldId id;

  GridMap map;

  // Physical state
  std::unordered_map<Agent, Pose> poses;

  // Intentional state (multi-agent planning)
  std::unordered_map<Agent, std::string> goals;

  /**
   * Equality is structural: used for world merging.
   */
  bool operator==(const World& other) const {
    return id == other.id;
  }
};

} // namespace epistemic
