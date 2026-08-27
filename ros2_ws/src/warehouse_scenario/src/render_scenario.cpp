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
// Draws each result file over the warehouse it was computed on: the floor,
// the zones, the route and the cells the robot had to sense from.
//
// PPM, because it is a header and bytes and needs no library; run_demo.sh
// converts it to PNG when the netpbm tools are around. A picture of a route
// is worth having in the repository, and worth not adding a dependency for.

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "warehouse_scenario/warehouse.hpp"

namespace fs = std::filesystem;
using namespace warehouse_scenario;

namespace
{

struct Colour
{
  uint8_t r, g, b;
};

constexpr Colour kFreeColour{250, 250, 248};
constexpr Colour kOccupiedColour{52, 56, 66};
constexpr Colour kUnknownColour{198, 204, 212};
constexpr Colour kRouteColour{31, 79, 216};
constexpr Colour kSensingColour{194, 24, 91};
constexpr Colour kSettledZone{31, 111, 74};
constexpr Colour kOpenZone{184, 134, 11};

constexpr int kScale = 3;   // image pixels per cell

class Canvas
{
public:
  Canvas()
  : width_(static_cast<int>(kWidth) * kScale),
    height_(static_cast<int>(kHeight) * kScale),
    pixels_(static_cast<std::size_t>(width_) * height_ * 3, 255) {}

  void set(int x, int y, const Colour & colour)
  {
    if (x < 0 || y < 0 || x >= width_ || y >= height_) {return;}
    // Row zero of the image is the top; row zero of the grid is the south.
    const std::size_t at = (static_cast<std::size_t>(height_ - 1 - y) * width_ + x) * 3;
    pixels_[at] = colour.r;
    pixels_[at + 1] = colour.g;
    pixels_[at + 2] = colour.b;
  }

  void disc(double mx, double my, int radius, const Colour & colour)
  {
    const int cx = static_cast<int>(mx / kResolution * kScale);
    const int cy = static_cast<int>(my / kResolution * kScale);
    for (int dy = -radius; dy <= radius; ++dy) {
      for (int dx = -radius; dx <= radius; ++dx) {
        if (dx * dx + dy * dy <= radius * radius) {set(cx + dx, cy + dy, colour);}
      }
    }
  }

  /// The outline of a metric rectangle, so a zone shows without hiding the
  /// floor under it.
  void frame(const Box & box, const Colour & colour)
  {
    const int x0 = static_cast<int>(box.min_x / kResolution * kScale);
    const int x1 = static_cast<int>(box.max_x / kResolution * kScale);
    const int y0 = static_cast<int>(box.min_y / kResolution * kScale);
    const int y1 = static_cast<int>(box.max_y / kResolution * kScale);
    for (int x = x0; x <= x1; ++x) {
      set(x, y0, colour);
      set(x, y1, colour);
    }
    for (int y = y0; y <= y1; ++y) {
      set(x0, y, colour);
      set(x1, y, colour);
    }
  }

  void write(const fs::path & path) const
  {
    std::ofstream out(path, std::ios::binary);
    out << "P6\n" << width_ << " " << height_ << "\n255\n";
    out.write(reinterpret_cast<const char *>(pixels_.data()),
      static_cast<std::streamsize>(pixels_.size()));
  }

private:
  int width_;
  int height_;
  std::vector<uint8_t> pixels_;
};

void draw_floor(Canvas & canvas, const std::vector<int8_t> & grid)
{
  for (uint32_t row = 0; row < kHeight; ++row) {
    for (uint32_t col = 0; col < kWidth; ++col) {
      const int8_t value = grid[static_cast<std::size_t>(row) * kWidth + col];
      const Colour colour = value == kFree ? kFreeColour :
        value == kOccupied ? kOccupiedColour : kUnknownColour;
      for (int dy = 0; dy < kScale; ++dy) {
        for (int dx = 0; dx < kScale; ++dx) {
          canvas.set(static_cast<int>(col) * kScale + dx,
            static_cast<int>(row) * kScale + dy, colour);
        }
      }
    }
  }
}

void draw_route(Canvas & canvas, const nlohmann::json & points, int radius,
  const Colour & colour)
{
  for (const auto & point : points) {
    canvas.disc(point.at(0).get<double>(), point.at(1).get<double>(), radius, colour);
  }
}

bool render(const fs::path & result_file, const fs::path & out_dir)
{
  std::ifstream in(result_file);
  nlohmann::json result;
  try {
    in >> result;
  } catch (const nlohmann::json::exception & error) {
    std::cerr << result_file << ": " << error.what() << "\n";
    return false;
  }
  if (!result.contains("case")) {return false;}

  Canvas canvas;
  draw_floor(canvas, build_grid(result.value("east_corridor_observed", true)));

  canvas.frame(kDock1, kSettledZone);
  canvas.frame(kDock2, kSettledZone);
  if (result.value("snapshot", "") != "docks") {
    canvas.frame(kBayAisle2, kOpenZone);
    canvas.frame(kBayAisle3, kOpenZone);
  }

  draw_route(canvas, result.value("path", nlohmann::json::array()), 2, kRouteColour);
  draw_route(canvas, result.value("sensing", nlohmann::json::array()), 3, kSensingColour);

  canvas.disc(kR1Start.x, kR1Start.y, 4, Colour{40, 40, 40});
  canvas.disc(kR2Start.x, kR2Start.y, 4, Colour{40, 40, 40});

  const std::string stem = result_file.stem().string();
  canvas.write(out_dir / (stem + ".ppm"));
  return true;
}

}  // namespace

int main(int argc, char ** argv)
{
  const fs::path root = argc > 1 ? fs::path(argv[1]) : fs::path("scenarios/warehouse");
  const fs::path out = root / "out";
  if (!fs::exists(out)) {
    std::cerr << "no results under " << fs::absolute(out) << "\n";
    return 1;
  }

  std::vector<fs::path> files;
  for (const auto & entry : fs::directory_iterator(out)) {
    if (entry.path().extension() == ".json") {files.push_back(entry.path());}
  }
  std::sort(files.begin(), files.end());

  int drawn = 0;
  for (const auto & file : files) {drawn += render(file, out) ? 1 : 0;}

  std::cout << "rendered " << drawn << " result(s) into " << fs::absolute(out) << "\n";
  return 0;
}
