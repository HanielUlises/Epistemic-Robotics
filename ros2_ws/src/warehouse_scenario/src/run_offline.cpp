// Copyright 2026 Haniel Vásquez Morales
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Runs every case of the warehouse scenario with no ROS in the way: the same
// resolve_query and the same fixed point the node runs, called directly.
//
// This is the reproducible half of the scenario. The ROS driver beside it
// proves the wiring; this proves the answer, and it does so without a
// discovery daemon, a timeout or a second process.

#include <filesystem>
#include <iomanip>
#include <iostream>
#include <string>

#include "scenario_io.hpp"
#include "warehouse_scenario/warehouse.hpp"

namespace fs = std::filesystem;
using namespace warehouse_scenario;

int main(int argc, char ** argv)
{
  const fs::path root = argc > 1 ? fs::path(argv[1]) : fs::path("scenarios/warehouse");
  const std::string only = argc > 2 ? argv[2] : "";

  int failures = 0;
  std::cout << std::left
            << std::setw(24) << "case"
            << std::setw(7) << "goal"
            << std::setw(7) << "known"
            << std::setw(9) << "disputed"
            << std::setw(9) << "sensing"
            << std::setw(9) << "region"
            << std::setw(7) << "iters"
            << std::setw(7) << "path"
            << "look\n";

  for (const auto & c : cases()) {
    if (!only.empty() && c.name != only) {continue;}

    const Outcome outcome = run(c);
    if (!outcome.ok) {
      std::cout << std::setw(24) << c.name << "FAILED: " << outcome.error << "\n";
      ++failures;
      continue;
    }

    std::cout << std::setw(24) << c.name
              << std::setw(7) << outcome.goal_cells
              << std::setw(7) << outcome.known_goal_cells
              << std::setw(9) << outcome.disputed_cells
              << std::setw(9) << outcome.sensing_cells
              << std::setw(9) << outcome.winning_region
              << std::setw(7) << outcome.iterations
              << std::setw(7) << outcome.path.size()
              << outcome.sensing_waypoints.size() << "\n";

    nlohmann::json extra;
    extra["goal_cells"] = outcome.goal_cells;
    extra["known_goal_cells"] = outcome.known_goal_cells;
    extra["disputed_cells"] = outcome.disputed_cells;
    extra["sensing_cells"] = outcome.sensing_cells;
    extra["safe_cells"] = outcome.safe_cells;
    extra["winning_region"] = outcome.winning_region;
    extra["iterations"] = outcome.iterations;

    write_result(root / "out", c.name, to_json(c, route_of(outcome), "offline", extra));
  }

  std::cout << "\nresults under " << fs::absolute(root / "out") << "\n";
  return failures == 0 ? 0 : 1;
}
