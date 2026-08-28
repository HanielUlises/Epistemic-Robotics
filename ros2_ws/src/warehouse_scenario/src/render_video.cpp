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
// Animates the warehouse scenario: one segment per case, the robot walking
// the route the fixed point returned, stopping where it has to look.
//
// Frames go to stdout as raw RGB24 and are encoded by ffmpeg, so a thirty
// second film never lands on disk as a thousand bitmaps. The captions are
// drawn by ffmpeg too, from the segment table this writes with --segments:
// text is the one thing a hand-rolled renderer does badly and a font engine
// does well.

#include <algorithm>
#include <cmath>
#include <cstdio>
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

// -- geometry ---------------------------------------------------------------

constexpr int kScale = 3;                       // image pixels per cell
constexpr int kMapWidth = static_cast<int>(kWidth) * kScale;    // 843
constexpr int kMapHeight = static_cast<int>(kHeight) * kScale;  // 1263
constexpr int kBand = 80;                       // the caption band, at the top
constexpr int kFrameWidth = kMapWidth;
constexpr int kFrameHeight = kMapHeight + kBand;

constexpr int kFps = 30;

// -- timing, in frames ------------------------------------------------------

constexpr int kOpening = 24;      // the floor, before anything moves
constexpr int kWalking = 96;      // the route, however long it is
constexpr int kLooking = 36;      // stopped, sensing
constexpr int kSettling = 24;     // after the pallet is revealed
constexpr int kClosing = 30;      // the result, held

// -- colour -----------------------------------------------------------------

struct Colour
{
  uint8_t r, g, b;
};

constexpr Colour kBandColour{24, 27, 33};
constexpr Colour kFreeColour{250, 250, 248};
constexpr Colour kOccupiedColour{52, 56, 66};
constexpr Colour kUnknownColour{176, 184, 196};
constexpr Colour kTrailColour{31, 79, 216};
constexpr Colour kRobotColour{16, 22, 40};
constexpr Colour kSensingColour{194, 24, 91};
constexpr Colour kSettledZone{31, 111, 74};
constexpr Colour kOpenZone{184, 134, 11};
constexpr Colour kPalletColour{219, 152, 30};
constexpr Colour kBlockedColour{170, 30, 30};

Colour blend(const Colour & over, const Colour & under, double alpha)
{
  const auto mix = [alpha](uint8_t a, uint8_t b) {
      return static_cast<uint8_t>(std::lround(a * alpha + b * (1.0 - alpha)));
    };
  return Colour{mix(over.r, under.r), mix(over.g, under.g), mix(over.b, under.b)};
}

// -- canvas -----------------------------------------------------------------

class Canvas
{
public:
  Canvas()
  : pixels_(static_cast<std::size_t>(kFrameWidth) * kFrameHeight * 3, 0) {}

  void set(int x, int y, const Colour & colour)
  {
    if (x < 0 || y < 0 || x >= kFrameWidth || y >= kFrameHeight) {return;}
    const std::size_t at = (static_cast<std::size_t>(y) * kFrameWidth + x) * 3;
    pixels_[at] = colour.r;
    pixels_[at + 1] = colour.g;
    pixels_[at + 2] = colour.b;
  }

  Colour get(int x, int y) const
  {
    if (x < 0 || y < 0 || x >= kFrameWidth || y >= kFrameHeight) {return kFreeColour;}
    const std::size_t at = (static_cast<std::size_t>(y) * kFrameWidth + x) * 3;
    return Colour{pixels_[at], pixels_[at + 1], pixels_[at + 2]};
  }

  /// Map metres to image pixels. The band sits above the map, and the map is
  /// drawn with north up, which is the way round a warehouse is looked at.
  void at_metres(double mx, double my, int & x, int & y) const
  {
    x = static_cast<int>(std::lround((mx - kOriginX) / kResolution * kScale));
    y = kBand + (kMapHeight - 1 -
      static_cast<int>(std::lround((my - kOriginY) / kResolution * kScale)));
  }

  void disc(double mx, double my, double radius, const Colour & colour, double alpha = 1.0)
  {
    int cx = 0, cy = 0;
    at_metres(mx, my, cx, cy);
    const int r = static_cast<int>(std::ceil(radius));
    for (int dy = -r; dy <= r; ++dy) {
      for (int dx = -r; dx <= r; ++dx) {
        const double distance = std::sqrt(static_cast<double>(dx * dx + dy * dy));
        if (distance > radius) {continue;}
        // A single pixel of feathering at the rim, so a moving dot does not
        // crawl from one cell to the next in steps.
        const double edge = std::clamp(radius - distance, 0.0, 1.0);
        set(cx + dx, cy + dy, blend(colour, get(cx + dx, cy + dy), alpha * edge));
      }
    }
  }

  void ring(double mx, double my, double radius, const Colour & colour, double alpha)
  {
    int cx = 0, cy = 0;
    at_metres(mx, my, cx, cy);
    const int r = static_cast<int>(std::ceil(radius)) + 1;
    for (int dy = -r; dy <= r; ++dy) {
      for (int dx = -r; dx <= r; ++dx) {
        const double distance = std::sqrt(static_cast<double>(dx * dx + dy * dy));
        if (std::abs(distance - radius) > 1.2) {continue;}
        set(cx + dx, cy + dy, blend(colour, get(cx + dx, cy + dy), alpha));
      }
    }
  }

  void frame_box(const Box & box, const Colour & colour, int thickness = 2)
  {
    int x0 = 0, y0 = 0, x1 = 0, y1 = 0;
    at_metres(box.min_x, box.min_y, x0, y0);
    at_metres(box.max_x, box.max_y, x1, y1);
    for (int t = 0; t < thickness; ++t) {
      for (int x = x0; x <= x1; ++x) {
        set(x, y0 + t, colour);
        set(x, y1 - t, colour);
      }
      for (int y = y1; y <= y0; ++y) {
        set(x0 + t, y, colour);
        set(x1 - t, y, colour);
      }
    }
  }

  void fill_box(const Box & box, const Colour & colour, double alpha)
  {
    int x0 = 0, y0 = 0, x1 = 0, y1 = 0;
    at_metres(box.min_x, box.min_y, x0, y0);
    at_metres(box.max_x, box.max_y, x1, y1);
    for (int y = y1; y <= y0; ++y) {
      for (int x = x0; x <= x1; ++x) {
        set(x, y, blend(colour, get(x, y), alpha));
      }
    }
  }

  void write(std::FILE * out) const
  {
    std::fwrite(pixels_.data(), 1, pixels_.size(), out);
  }

  void draw_floor(const std::vector<int8_t> & grid)
  {
    for (int y = 0; y < kBand; ++y) {
      for (int x = 0; x < kFrameWidth; ++x) {set(x, y, kBandColour);}
    }
    for (uint32_t row = 0; row < kHeight; ++row) {
      for (uint32_t col = 0; col < kWidth; ++col) {
        const int8_t value = grid[static_cast<std::size_t>(row) * kWidth + col];
        const Colour colour = value == kFree ? kFreeColour :
          value == kOccupied ? kOccupiedColour : kUnknownColour;
        const int y0 = kBand + (kMapHeight - 1 - static_cast<int>(row) * kScale);
        for (int dy = 0; dy < kScale; ++dy) {
          for (int dx = 0; dx < kScale; ++dx) {
            set(static_cast<int>(col) * kScale + dx, y0 - dy, colour);
          }
        }
      }
    }
  }

private:
  std::vector<uint8_t> pixels_;
};

// -- one segment ------------------------------------------------------------

/// A case, its answer, and the frames it takes to tell.
struct Segment
{
  Case scenario;
  Outcome outcome;
  int first_frame{0};
  int frames{0};
  std::string headline;
  std::string caption;
};

std::string headline_of(const Case & c, const Outcome & outcome)
{
  if (outcome.path.empty()) {
    return c.goal_zone + ": no route";
  }
  std::string text = c.goal_zone + ": " + std::to_string(outcome.path.size()) + " cells";
  if (!outcome.sensing_waypoints.empty()) {
    text += ", " + std::to_string(outcome.sensing_waypoints.size()) + " stop to look";
  }
  return text;
}

std::vector<Segment> plan_segments()
{
  std::vector<Segment> segments;
  int frame = 0;

  for (const auto & c : cases()) {
    Segment segment;
    segment.scenario = c;
    segment.outcome = run(c);
    segment.first_frame = frame;

    segment.frames = kOpening + kClosing +
      (segment.outcome.path.empty() ? kWalking / 2 : kWalking) +
      (segment.outcome.sensing_waypoints.empty() ? 0 : kLooking + kSettling);

    segment.headline = headline_of(c, segment.outcome);
    segment.caption = c.expect;

    frame += segment.frames;
    segments.push_back(std::move(segment));
  }
  return segments;
}

// -- drawing one frame ------------------------------------------------------

/// Where along the route the robot is, and whether it has stopped to look.
struct Phase
{
  double progress{0.0};     ///< 0 to 1 along the path
  bool looking{false};
  double look_pulse{0.0};   ///< 0 to 1 within the look
  bool revealed{false};     ///< the sensing action has settled the bay
};

Phase phase_of(const Segment & segment, int local_frame)
{
  Phase phase;
  const bool senses = !segment.outcome.sensing_waypoints.empty();
  const int walking = segment.outcome.path.empty() ? kWalking / 2 : kWalking;

  int at = local_frame;
  if (at < kOpening) {return phase;}
  at -= kOpening;

  if (at < walking) {
    phase.progress = walking > 1 ? static_cast<double>(at) / (walking - 1) : 1.0;
    return phase;
  }
  at -= walking;
  phase.progress = 1.0;

  if (senses) {
    if (at < kLooking) {
      phase.looking = true;
      phase.look_pulse = static_cast<double>(at) / kLooking;
      return phase;
    }
    at -= kLooking;
    phase.revealed = true;
    if (at < kSettling) {return phase;}
  }
  phase.revealed = senses;
  return phase;
}

void draw(Canvas & canvas, const Segment & segment, int local_frame)
{
  const auto & c = segment.scenario;
  const auto & outcome = segment.outcome;
  const Phase phase = phase_of(segment, local_frame);

  canvas.draw_floor(build_grid(c.stage));

  canvas.frame_box(kDockSouth, kSettledZone);
  canvas.frame_box(kDockNorth, kSettledZone);
  canvas.frame_box(kBayAisle4, kSettledZone);

  const bool bays_in_play = c.snapshot != "docks";
  if (bays_in_play) {
    canvas.frame_box(kBayAisle2, kOpenZone);
    canvas.frame_box(kBayAisle3, kOpenZone);

    // Until something settles it, the pallet is in both bays and in neither:
    // the two worlds disagree, and the picture says so by drawing both faintly
    // rather than by picking one and calling it the truth.
    const double faint = phase.revealed ? 0.0 : 0.28;
    canvas.fill_box(kBayAisle2, kPalletColour, faint);
    canvas.fill_box(kBayAisle3, kPalletColour, faint);

    if (phase.revealed) {
      // w0 is the designated world, and it puts the pallet in aisle 2.
      canvas.fill_box(kBayAisle2, kPalletColour, 0.85);
      canvas.fill_box(kBayAisle3, kFreeColour, 0.85);
      canvas.frame_box(kBayAisle3, kOpenZone);
    }
  }

  // The route, drawn as far as the robot has come.
  if (!outcome.path.empty()) {
    const std::size_t last = static_cast<std::size_t>(
      std::lround(phase.progress * (outcome.path.size() - 1)));
    for (std::size_t i = 0; i <= last; ++i) {
      const Point at = point_of(outcome.path[i]);
      canvas.disc(at.x, at.y, 2.2, kTrailColour, 0.55);
    }

    const Point head = point_of(outcome.path[last]);
    canvas.disc(head.x, head.y, 7.0, kRobotColour, 0.95);
    canvas.disc(head.x, head.y, 3.0, kFreeColour, 0.9);

    if (phase.looking) {
      // A ring going out from where it stands: the look, in the only place
      // from which the question can be answered.
      const double radius = 8.0 + 26.0 * phase.look_pulse;
      canvas.ring(head.x, head.y, radius, kSensingColour, 1.0 - phase.look_pulse);
      canvas.ring(head.x, head.y, radius * 0.6, kSensingColour, 0.6 - 0.5 * phase.look_pulse);
    }
  } else {
    // No route: the robot stays where it is, and the aisle it has never
    // looked down is the reason. Marked rather than narrated.
    const Point start = point_of(outcome.start);
    canvas.disc(start.x, start.y, 7.0, kRobotColour, 0.95);
    canvas.disc(start.x, start.y, 3.0, kFreeColour, 0.9);

    const double pulse = 0.35 + 0.25 * std::sin(local_frame * 0.18);
    canvas.fill_box(kBayAisle4, kBlockedColour, pulse * 0.45);
  }

  for (const CellIdx cell : outcome.sensing_waypoints) {
    const Point at = point_of(cell);
    canvas.ring(at.x, at.y, 9.0, kSensingColour, 0.9);
  }
}

// -- the segment table ffmpeg reads ----------------------------------------

void write_segments(const std::vector<Segment> & segments, const fs::path & path)
{
  std::ofstream out(path);
  out << "# start_seconds end_seconds | case | headline | caption\n";
  for (const auto & segment : segments) {
    const double start = static_cast<double>(segment.first_frame) / kFps;
    const double end = static_cast<double>(segment.first_frame + segment.frames) / kFps;
    out << start << " " << end << "|" << segment.scenario.name << "|"
        << segment.headline << "|" << segment.caption << "\n";
  }
}

}  // namespace

int main(int argc, char ** argv)
{
  std::string segments_path;
  bool frames_to_stdout = true;

  for (int i = 1; i < argc; ++i) {
    const std::string argument = argv[i];
    if (argument == "--segments" && i + 1 < argc) {
      segments_path = argv[++i];
      frames_to_stdout = false;
    } else if (argument == "--size") {
      std::cout << kFrameWidth << "x" << kFrameHeight << " " << kFps << "\n";
      return 0;
    }
  }

  const auto segments = plan_segments();

  if (!segments_path.empty()) {
    write_segments(segments, segments_path);
    int total = 0;
    for (const auto & segment : segments) {total += segment.frames;}
    std::cerr << segments.size() << " segments, " << total << " frames, "
              << static_cast<double>(total) / kFps << " s\n";
    return 0;
  }

  if (frames_to_stdout) {
    Canvas canvas;
    for (const auto & segment : segments) {
      for (int local = 0; local < segment.frames; ++local) {
        draw(canvas, segment, local);
        canvas.write(stdout);
      }
    }
    std::fflush(stdout);
  }
  return 0;
}
