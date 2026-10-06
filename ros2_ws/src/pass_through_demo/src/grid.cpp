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

#include "pass_through_demo/grid.hpp"

#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace pass_through
{

std::map<std::string, Box> boxes_from(
  const std::vector<std::string> & names, const std::vector<double> & flat)
{
  if (flat.size() != 4 * names.size()) {
    throw std::runtime_error(
            "boxes: " + std::to_string(names.size()) + " names and " +
            std::to_string(flat.size()) + " numbers; four numbers per name");
  }
  std::map<std::string, Box> out;
  for (std::size_t i = 0; i < names.size(); ++i) {
    out[names[i]] = Box{flat[4 * i], flat[4 * i + 1], flat[4 * i + 2], flat[4 * i + 3]};
  }
  return out;
}

bool Grid::index(double x, double y, std::size_t & out) const
{
  const double cx = std::floor((x - origin_x) / resolution);
  const double cy = std::floor((y - origin_y) / resolution);
  if (cx < 0 || cy < 0 || cx >= width || cy >= height) {
    return false;
  }
  out = static_cast<std::size_t>(cy) * width + static_cast<std::size_t>(cx);
  return true;
}

void Grid::centre(std::size_t i, double & x, double & y) const
{
  x = origin_x + (static_cast<double>(i % width) + 0.5) * resolution;
  y = origin_y + (static_cast<double>(i / width) + 0.5) * resolution;
}

std::vector<std::size_t> Grid::cells_in(const Box & box) const
{
  std::vector<std::size_t> out;
  if (empty()) {
    return out;
  }
  const auto c0 = static_cast<long>(std::floor((box.min_x - origin_x) / resolution));
  const auto c1 = static_cast<long>(std::floor((box.max_x - origin_x) / resolution));
  const auto r0 = static_cast<long>(std::floor((box.min_y - origin_y) / resolution));
  const auto r1 = static_cast<long>(std::floor((box.max_y - origin_y) / resolution));
  for (long r = std::max(0L, r0); r <= std::min<long>(height - 1, r1); ++r) {
    for (long c = std::max(0L, c0); c <= std::min<long>(width - 1, c1); ++c) {
      const std::size_t i = static_cast<std::size_t>(r) * width + static_cast<std::size_t>(c);
      double x, y;
      centre(i, x, y);
      if (box.contains(x, y)) {
        out.push_back(i);
      }
    }
  }
  return out;
}

bool Grid::same_geometry(const Grid & other) const
{
  // The resolution crosses a message as a float32, so 0.1 comes back as
  // 0.100000001. A tolerance of a micrometre accepts that and still refuses a
  // grid that is genuinely different.
  return width == other.width && height == other.height &&
         std::fabs(resolution - other.resolution) < 1e-6 &&
         std::fabs(origin_x - other.origin_x) < 1e-6 &&
         std::fabs(origin_y - other.origin_y) < 1e-6;
}

Grid blank_like(const Grid & geometry, std::int8_t value)
{
  Grid out = geometry;
  out.cells.assign(static_cast<std::size_t>(geometry.width) * geometry.height, value);
  return out;
}

namespace
{

std::string trim(const std::string & text)
{
  const auto a = text.find_first_not_of(" \t\r\n");
  const auto b = text.find_last_not_of(" \t\r\n");
  return a == std::string::npos ? std::string{} : text.substr(a, b - a + 1);
}

/// The next token of a PGM header, skipping comments.
std::string pgm_token(std::istream & in)
{
  std::string token;
  while (in >> token) {
    if (token[0] == '#') {
      std::string rest;
      std::getline(in, rest);
      continue;
    }
    return token;
  }
  throw std::runtime_error("truncated PGM header");
}

}  // namespace

Grid load_map_yaml(const std::string & yaml_path)
{
  std::ifstream yaml(yaml_path);
  if (!yaml) {
    throw std::runtime_error("cannot read " + yaml_path);
  }

  // The file is the handful of keys map_server writes. A YAML library for
  // six scalar keys would be a dependency with nothing to do.
  std::string image;
  Grid grid;
  double occupied_thresh = 0.65, free_thresh = 0.25;
  for (std::string line; std::getline(yaml, line); ) {
    const auto colon = line.find(':');
    if (colon == std::string::npos) {
      continue;
    }
    const auto key = trim(line.substr(0, colon));
    const auto value = trim(line.substr(colon + 1));
    if (key == "image") {
      image = value;
    } else if (key == "resolution") {
      grid.resolution = std::stod(value);
    } else if (key == "origin") {
      std::string numbers = value;
      for (auto & ch : numbers) {
        if (ch == '[' || ch == ']' || ch == ',') {ch = ' ';}
      }
      std::istringstream(numbers) >> grid.origin_x >> grid.origin_y;
    } else if (key == "occupied_thresh") {
      occupied_thresh = std::stod(value);
    } else if (key == "free_thresh") {
      free_thresh = std::stod(value);
    }
  }
  if (image.empty()) {
    throw std::runtime_error(yaml_path + " names no image");
  }
  if (image[0] != '/') {
    const auto slash = yaml_path.find_last_of('/');
    image = (slash == std::string::npos ? std::string{} : yaml_path.substr(0, slash + 1)) + image;
  }

  std::ifstream pgm(image, std::ios::binary);
  if (!pgm) {
    throw std::runtime_error("cannot read " + image);
  }
  if (pgm_token(pgm) != "P5") {
    throw std::runtime_error(image + " is not a binary PGM");
  }
  grid.width = static_cast<std::uint32_t>(std::stoul(pgm_token(pgm)));
  grid.height = static_cast<std::uint32_t>(std::stoul(pgm_token(pgm)));
  const int maxval = std::stoi(pgm_token(pgm));
  pgm.get();   // the single whitespace byte before the raster

  std::vector<unsigned char> raster(static_cast<std::size_t>(grid.width) * grid.height);
  pgm.read(reinterpret_cast<char *>(raster.data()), static_cast<std::streamsize>(raster.size()));
  if (!pgm) {
    throw std::runtime_error(image + " is shorter than its header says");
  }

  // Trinary, as map_server reads it: darkness is occupancy, and the band
  // between the two thresholds is unknown. PGM rows run north to south.
  grid.cells.assign(raster.size(), -1);
  for (std::uint32_t r = 0; r < grid.height; ++r) {
    for (std::uint32_t c = 0; c < grid.width; ++c) {
      const double occ =
        (maxval - raster[static_cast<std::size_t>(r) * grid.width + c]) / static_cast<double>(maxval);
      std::int8_t value = -1;
      if (occ > occupied_thresh) {
        value = 100;
      } else if (occ < free_thresh) {
        value = 0;
      }
      grid.cells[static_cast<std::size_t>(grid.height - 1 - r) * grid.width + c] = value;
    }
  }
  return grid;
}

Grid from_msg(const nav_msgs::msg::OccupancyGrid & msg)
{
  Grid grid;
  grid.width = msg.info.width;
  grid.height = msg.info.height;
  grid.resolution = msg.info.resolution;
  grid.origin_x = msg.info.origin.position.x;
  grid.origin_y = msg.info.origin.position.y;
  grid.cells.assign(msg.data.begin(), msg.data.end());
  return grid;
}

nav_msgs::msg::OccupancyGrid to_msg(const Grid & grid, const std::string & frame)
{
  nav_msgs::msg::OccupancyGrid msg;
  msg.header.frame_id = frame;
  msg.info.width = grid.width;
  msg.info.height = grid.height;
  msg.info.resolution = static_cast<float>(grid.resolution);
  msg.info.origin.position.x = grid.origin_x;
  msg.info.origin.position.y = grid.origin_y;
  msg.info.origin.orientation.w = 1.0;
  msg.data.assign(grid.cells.begin(), grid.cells.end());
  return msg;
}

Grid resample(const Grid & source, const Grid & geometry)
{
  Grid out = blank_like(geometry, -1);
  if (source.empty()) {
    return out;
  }
  for (std::size_t i = 0; i < out.size(); ++i) {
    double x, y;
    out.centre(i, x, y);
    std::size_t j;
    if (source.index(x, y, j)) {
      out.cells[i] = source.cells[j];
    }
  }
  return out;
}

const char * to_string(Verdict verdict)
{
  switch (verdict) {
    case Verdict::Clear: return "clear";
    case Verdict::Blocked: return "blocked";
    default: return "unknown";
  }
}

RegionReading read_region(
  const Grid & grid, const Box & box, const epistemic_slam::Thresholds & thresholds)
{
  RegionReading reading;
  for (const auto i : grid.cells_in(box)) {
    ++reading.cells;
    switch (epistemic_slam::classify(grid.cells[i], thresholds)) {
      case epistemic_slam::CellClass::Free: ++reading.free; break;
      case epistemic_slam::CellClass::Occupied: ++reading.occupied; break;
      default: ++reading.unknown; break;
    }
  }
  if (reading.occupied > 0) {
    reading.verdict = Verdict::Blocked;
  } else if (reading.cells > 0 && reading.free == reading.cells) {
    reading.verdict = Verdict::Clear;
  }
  return reading;
}

}  // namespace pass_through
