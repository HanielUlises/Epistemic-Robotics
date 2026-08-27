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

#pragma once

#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "warehouse_scenario/warehouse.hpp"

// One result file shape, written by the offline run and by the ROS driver
// alike, so that the renderer draws either without knowing which produced it
// and the two can be compared line by line.

namespace warehouse_scenario
{

/// A route in map metres, which is what both producers have in common: the
/// offline run holds cells, the driver receives poses.
struct Route
{
  std::vector<Point> path;
  std::vector<Point> sensing;
};

inline nlohmann::json to_json(const Case & c, const Route & route,
  const std::string & source, const nlohmann::json & extra = nlohmann::json::object())
{
  nlohmann::json result;
  result["case"] = c.name;
  result["source"] = source;
  result["east_corridor_observed"] = c.east_corridor_observed;
  result["snapshot"] = c.snapshot;
  result["agent_id"] = c.agent_id;
  result["goal_zone"] = c.goal_zone;
  result["require_epistemic_goal"] = c.require_epistemic_goal;
  result["expect"] = c.expect;
  result["path_length"] = route.path.size();
  result["sensing_length"] = route.sensing.size();

  for (const auto & p : route.path) {result["path"].push_back({p.x, p.y});}
  for (const auto & p : route.sensing) {result["sensing"].push_back({p.x, p.y});}
  if (route.path.empty()) {result["path"] = nlohmann::json::array();}
  if (route.sensing.empty()) {result["sensing"] = nlohmann::json::array();}

  for (const auto & [key, value] : extra.items()) {result[key] = value;}
  return result;
}

inline void write_result(const std::filesystem::path & directory,
  const std::string & stem, const nlohmann::json & result)
{
  std::filesystem::create_directories(directory);
  std::ofstream out(directory / (stem + ".json"));
  out << result.dump(2) << "\n";
}

inline Route route_of(const Outcome & outcome)
{
  Route route;
  route.path.reserve(outcome.path.size());
  for (const CellIdx cell : outcome.path) {route.path.push_back(point_of(cell));}
  for (const CellIdx cell : outcome.sensing_waypoints) {
    route.sensing.push_back(point_of(cell));
  }
  return route;
}

}  // namespace warehouse_scenario
