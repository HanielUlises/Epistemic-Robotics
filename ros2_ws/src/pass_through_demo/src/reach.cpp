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

#include "pass_through_demo/reach.hpp"

#include <algorithm>
#include <cmath>
#include <deque>

#include <nlohmann/json.hpp>

#include "mu_path_planner/epistemic_state.hpp"

namespace pass_through
{

namespace mpp = mu_path_planner;

namespace
{

/// The route formula's safety constraint, as plank formula JSON:
///     free  v  OR_t ( t ^ [agent] open_t )
std::string safety_formula(const std::map<std::string, Box> & bays, const std::string & agent)
{
  nlohmann::json disjuncts = nlohmann::json::array();
  disjuncts.push_back("free");
  for (const auto & [name, box] : bays) {
    (void)box;
    disjuncts.push_back({
        {"connective", "and"},
        {"formulas", {
            name,
            {{"modality-name", "box"}, {"modality-index", {agent}}, {"formula", "open_" + name}}}}});
  }
  return nlohmann::json{{"connective", "or"}, {"formulas", disjuncts}}.dump();
}

std::string safety_text(const std::map<std::string, Box> & bays, const std::string & agent)
{
  std::string text = "free";
  for (const auto & [name, box] : bays) {
    (void)box;
    text += " v (" + name + " ^ K_" + agent + " open_" + name + ")";
  }
  return text;
}

/// Grow a mask by a disc.
std::vector<char> dilate(const Grid & g, const std::vector<char> & mask, double metres)
{
  const int reach = static_cast<int>(std::ceil(metres / g.resolution));
  std::vector<std::pair<int, int>> disc;
  for (int dr = -reach; dr <= reach; ++dr) {
    for (int dc = -reach; dc <= reach; ++dc) {
      if (dr * dr + dc * dc <= reach * reach) {
        disc.emplace_back(dr, dc);
      }
    }
  }
  std::vector<char> out(mask.size(), 0);
  const int w = static_cast<int>(g.width), h = static_cast<int>(g.height);
  for (int r = 0; r < h; ++r) {
    for (int c = 0; c < w; ++c) {
      if (!mask[static_cast<std::size_t>(r) * w + c]) {continue;}
      for (const auto & [dr, dc] : disc) {
        const int nr = r + dr, nc = c + dc;
        if (nr >= 0 && nc >= 0 && nr < h && nc < w) {
          out[static_cast<std::size_t>(nr) * w + nc] = 1;
        }
      }
    }
  }
  return out;
}

}  // namespace

SafeSet build_safe_set(const SafeSetInput & in)
{
  SafeSet out;
  if (!in.floorplan || in.floorplan->empty()) {
    out.error = "no floor plan";
    return out;
  }
  const Grid & plan = *in.floorplan;
  const bool have_knowledge = in.knowledge && in.knowledge->same_geometry(plan);
  const std::size_t n = plan.size();

  std::vector<char> in_bay(n, 0);
  for (const auto & [name, box] : in.bays) {
    (void)name;
    for (const auto i : plan.cells_in(box)) {
      in_bay[i] = 1;
    }
  }

  // What the agent's map says about each cell. Outside the bays the floor
  // plan, which is certain there and which every agent holds; inside them the
  // agent's own knowledge map, which is the only thing that can say anything
  // about a bay. A robot's SLAM map is not consulted outside the bays: it
  // records the other robots as obstacles wherever it happened to see them,
  // and a route planned round where a robot stood a minute ago is not safer.
  std::vector<mpp::CellState> state(n, mpp::CellState::Unknown);
  std::vector<char> occupied(n, 0);
  for (std::size_t i = 0; i < n; ++i) {
    const std::int8_t value = in_bay[i] ?
      (have_knowledge ? in.knowledge->cells[i] : std::int8_t{-1}) : plan.cells[i];
    switch (epistemic_slam::classify(value, in.thresholds)) {
      case epistemic_slam::CellClass::Free: state[i] = mpp::CellState::Free; break;
      case epistemic_slam::CellClass::Occupied:
        state[i] = mpp::CellState::Occupied;
        occupied[i] = 1;
        break;
      default: break;
    }
    // The floor plan's walls hold inside a bay too: they are its sides.
    if (plan.cells[i] >= in.thresholds.occupied_above) {
      occupied[i] = 1;
    }
  }
  for (const auto & box : in.closed) {
    for (const auto i : plan.cells_in(box)) {
      occupied[i] = 1;
    }
  }
  for (const auto & [kx, ky] : in.keep_out) {
    const double r = in.keep_out_radius;
    for (const auto i : plan.cells_in(Box{kx - r, ky - r, kx + r, ky + r})) {
      double cx, cy;
      plan.centre(i, cx, cy);
      if (std::hypot(cx - kx, cy - ky) <= r) {
        occupied[i] = 1;
      }
    }
  }
  const auto inflated = dilate(plan, occupied, in.inflation);

  // The formula, evaluated by the µ-calculus planner's own evaluator over the
  // model the state published, with each bay a zone in every world.
  out.formula = safety_text(in.bays, in.agent);
  std::string error;
  const auto formula = mpp::parse_formula(safety_formula(in.bays, in.agent), error);
  if (!formula) {
    out.error = "safety formula: " + error;
    return out;
  }

  mpp::GridInfo info;
  info.width = plan.width;
  info.height = plan.height;
  info.resolution = plan.resolution;
  info.origin_x = plan.origin_x;
  info.origin_y = plan.origin_y;

  nlohmann::json snapshot;
  if (!in.model_json.empty()) {
    try {
      snapshot = nlohmann::json::parse(in.model_json);
    } catch (const std::exception & e) {
      out.error = std::string("model: ") + e.what();
      return out;
    }
  } else {
    // No model yet: one world, nothing true in it, every relation reflexive.
    // K_i open_t is then false everywhere, which is what "nothing is known"
    // has to mean.
    snapshot = {{"worlds", {"w0"}}, {"designated", {"w0"}}, {"labels", {{"w0", nlohmann::json::array()}}},
      {"relations", nlohmann::json::object()}};
  }
  for (const auto & [name, box] : in.bays) {
    snapshot["zones"][name]["bounds"] = {
      {"min_x", box.min_x}, {"min_y", box.min_y}, {"max_x", box.max_x}, {"max_y", box.max_y}};
  }
  const auto parsed = mpp::parse_snapshot(snapshot.dump(), info);
  if (!parsed.ok) {
    out.error = "snapshot: " + parsed.error;
    return out;
  }

  out.graph.width = plan.width;
  out.graph.height = plan.height;
  out.graph.obstacle.assign(n, true);

  mpp::EvalContext ctx;
  ctx.snapshot = &parsed;
  ctx.graph = &out.graph;
  ctx.cell_state = &state;

  out.safe.assign(n, 0);
  std::map<std::string, bool> lifted;
  for (std::size_t i = 0; i < n; ++i) {
    if (inflated[i]) {continue;}
    // Free cells are safe by the first disjunct whatever the model says; only
    // the rest are worth handing to the evaluator.
    const bool safe = state[i] == mpp::CellState::Free ||
      (in_bay[i] && mpp::holds_at_designated(*formula, static_cast<mpp::CellIdx>(i), ctx));
    out.safe[i] = safe ? 1 : 0;
    if (safe && state[i] != mpp::CellState::Free) {
      for (const auto & [name, box] : in.bays) {
        double x, y;
        plan.centre(i, x, y);
        if (box.contains(x, y)) {lifted[name] = true;}
      }
    }
  }
  for (const auto & [name, yes] : lifted) {
    if (yes) {out.lifted.push_back(name);}
  }

  // Which bays the agent knows open, asked of the model directly, so the log
  // can say so even when the map already shows the bay free and nothing had
  // to be lifted.
  for (const auto & [name, box] : in.bays) {
    (void)box;
    auto known = mpp::parse_formula(
      nlohmann::json{{"modality-name", "box"}, {"modality-index", {in.agent}},
        {"formula", "open_" + name}}.dump(), error);
    if (known && mpp::holds_at_designated(*known, 0, ctx)) {
      out.known_open.push_back(name);
    }
  }

  for (std::size_t i = 0; i < n; ++i) {
    out.graph.obstacle[i] = !out.safe[i];
  }
  out.graph.build_adjacency();
  out.ok = true;
  return out;
}

Reach least_fixed_point(
  const mpp::OccupancyGraph & graph, const std::vector<char> & goal, const std::vector<char> & safe)
{
  Reach out;
  const std::size_t n = static_cast<std::size_t>(graph.width) * graph.height;
  out.layer.assign(n, -1);

  // Z_1 = goal.
  std::vector<std::size_t> entered;
  for (std::size_t i = 0; i < n && i < goal.size(); ++i) {
    if (goal[i]) {
      out.layer[i] = 1;
      entered.push_back(i);
    }
  }
  if (entered.empty()) {
    return out;
  }
  out.iterations = 1;
  out.size = entered.size();

  // Z_{k+1} = goal v (Safe ^ pre(Z_k)): only the cells that entered at k can
  // add anything at k + 1. `neighbours` is symmetric over non-obstacle cells,
  // so the successors of u are its predecessors as well.
  std::int32_t k = 1;
  while (!entered.empty()) {
    std::vector<std::size_t> next;
    for (const auto u : entered) {
      for (const auto v : graph.neighbours[u]) {
        if (out.layer[v] < 0 && safe[v]) {
          out.layer[v] = k + 1;
          next.push_back(v);
        }
      }
    }
    ++out.iterations;
    ++k;
    out.size += next.size();
    entered = std::move(next);
  }
  // The last iteration added nothing: Z_{k} = Z_{k-1} is how a fixed point is
  // recognised, and mu_reach counts that confirming step too.
  return out;
}

std::vector<std::size_t> descend(
  const mpp::OccupancyGraph & graph, const Reach & reach, std::size_t start)
{
  std::vector<std::size_t> path;
  if (!reach.contains(start)) {
    return path;
  }
  std::size_t at = start;
  path.push_back(at);
  while (reach.layer[at] > 1) {
    std::size_t best = at;
    for (const auto v : graph.neighbours[at]) {
      if (reach.contains(v) && reach.layer[v] < reach.layer[best]) {
        best = v;
      }
    }
    if (best == at) {
      break;   // cannot happen in a fixed point computed from these neighbours
    }
    at = best;
    path.push_back(at);
  }
  return path;
}

bool nearest_in(
  const Grid & g, const std::vector<char> & region, std::size_t from, std::size_t reach,
  std::size_t & out)
{
  if (from < region.size() && region[from]) {
    out = from;
    return true;
  }
  const long w = g.width, h = g.height;
  const long r0 = static_cast<long>(from) / w, c0 = static_cast<long>(from) % w;
  double best = 1e18;
  bool found = false;
  const long span = static_cast<long>(reach);
  for (long r = std::max(0L, r0 - span); r <= std::min(h - 1, r0 + span); ++r) {
    for (long c = std::max(0L, c0 - span); c <= std::min(w - 1, c0 + span); ++c) {
      const std::size_t i = static_cast<std::size_t>(r * w + c);
      if (!region[i]) {continue;}
      const double d = static_cast<double>((r - r0) * (r - r0) + (c - c0) * (c - c0));
      if (d < best) {
        best = d;
        out = i;
        found = true;
      }
    }
  }
  return found;
}

std::vector<std::size_t> shortcut(
  const Grid & g, const std::vector<char> & safe, const std::vector<std::size_t> & path)
{
  if (path.size() < 3) {
    return path;
  }
  const auto clear = [&](std::size_t a, std::size_t b) {
      double ax, ay, bx, by;
      g.centre(a, ax, ay);
      g.centre(b, bx, by);
      const double length = std::hypot(bx - ax, by - ay);
      const int steps = std::max(1, static_cast<int>(std::ceil(length / (0.5 * g.resolution))));
      for (int s = 0; s <= steps; ++s) {
        const double t = static_cast<double>(s) / steps;
        std::size_t i;
        if (!g.index(ax + t * (bx - ax), ay + t * (by - ay), i) || !safe[i]) {
          return false;
        }
      }
      return true;
    };

  std::vector<std::size_t> out{path.front()};
  std::size_t anchor = 0;
  while (anchor + 1 < path.size()) {
    // The furthest point still in sight of the anchor. A straight segment is
    // only as long as the fixed point's own corridor allows, so the route
    // stays inside the winning region it was extracted from.
    std::size_t far = anchor + 1;
    for (std::size_t j = path.size() - 1; j > anchor + 1; --j) {
      if (clear(path[anchor], path[j])) {
        far = j;
        break;
      }
    }
    out.push_back(path[far]);
    anchor = far;
  }
  return out;
}

std::vector<char> erode(const Grid & g, const std::vector<char> & mask)
{
  std::vector<char> out(mask.size(), 0);
  const long w = g.width, h = g.height;
  for (long r = 1; r + 1 < h; ++r) {
    for (long c = 1; c + 1 < w; ++c) {
      bool all = true;
      for (long dr = -1; dr <= 1 && all; ++dr) {
        for (long dc = -1; dc <= 1 && all; ++dc) {
          all = mask[static_cast<std::size_t>((r + dr) * w + c + dc)] != 0;
        }
      }
      out[static_cast<std::size_t>(r * w + c)] = all ? 1 : 0;
    }
  }
  return out;
}

}  // namespace pass_through
