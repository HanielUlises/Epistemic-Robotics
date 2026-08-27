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
// Writes the two stages of the warehouse where map_server and rviz can read
// them, and the three snapshots beside them. Nothing downstream needs these
// files — the scenario is built in memory — but a map on disk is what lets
// the same floor be driven in Gazebo, opened in rviz, or diffed after a
// change to the layout.

#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "warehouse_scenario/warehouse.hpp"

namespace fs = std::filesystem;
using namespace warehouse_scenario;

namespace
{

/// Writes the grid in the PGM convention map_server reads.
///
/// Row zero of an OccupancyGrid is the south edge and row zero of a PGM is
/// the top of the image, so the rows are flipped here and flipped back on the
/// way in. Getting this backwards mirrors the warehouse without saying so.
void write_pgm(const std::vector<int8_t> & grid, const fs::path & path)
{
  std::vector<uint8_t> image(grid.size(), 205);   // unknown
  for (std::size_t i = 0; i < grid.size(); ++i) {
    if (grid[i] == kFree) {
      image[i] = 254;
    } else if (grid[i] == kOccupied) {
      image[i] = 0;
    }
  }

  std::ofstream out(path, std::ios::binary);
  out << "P5\n" << kWidth << " " << kHeight << "\n255\n";
  for (uint32_t row = kHeight; row-- > 0; ) {
    out.write(reinterpret_cast<const char *>(&image[static_cast<std::size_t>(row) * kWidth]),
      kWidth);
  }
}

void write_yaml(const std::string & image_name, const fs::path & path)
{
  std::ofstream out(path);
  out << "image: " << image_name << "\n"
      << "resolution: " << kResolution << "\n"
      << "origin: [0.0, 0.0, 0.0]\n"
      << "negate: 0\n"
      << "occupied_thresh: 0.65\n"
      << "free_thresh: 0.25\n";
}

void write_text(const std::string & text, const fs::path & path)
{
  std::ofstream out(path);
  out << text;
}

void report(const std::string & name, const std::vector<int8_t> & grid)
{
  std::size_t free = 0, occupied = 0, unknown = 0;
  for (const int8_t value : grid) {
    if (value == kFree) {++free;} else if (value == kOccupied) {++occupied;} else {++unknown;}
  }
  std::cout << name << ": " << kWidth << "x" << kHeight << " cells at "
            << kResolution << " m  free=" << free << " occupied=" << occupied
            << " unknown=" << unknown << "\n";
}

}  // namespace

int main(int argc, char ** argv)
{
  const fs::path root = argc > 1 ? fs::path(argv[1]) : fs::path("scenarios/warehouse");
  const fs::path maps = root / "maps";
  const fs::path snapshots = root / "snapshots";
  fs::create_directories(maps);
  fs::create_directories(snapshots);

  for (const auto & [name, observed] :
    std::vector<std::pair<std::string, bool>>{{"stage_a", false}, {"stage_b", true}})
  {
    const auto grid = build_grid(observed);
    write_pgm(grid, maps / (name + ".pgm"));
    write_yaml(name + ".pgm", maps / (name + ".yaml"));
    report(name, grid);
  }

  write_text(snapshot_docks(), snapshots / "docks.json");
  write_text(snapshot_pallet(), snapshots / "pallet.json");
  write_text(snapshot_fleet(), snapshots / "fleet.json");
  std::cout << "snapshots: docks, pallet, fleet\n";

  std::cout << cases().size() << " cases\n";
  std::cout << "written under " << fs::absolute(root) << "\n";
  return 0;
}
