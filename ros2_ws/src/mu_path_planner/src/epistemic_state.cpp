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

#include "mu_path_planner/epistemic_state.hpp"

#include <algorithm>
#include <cmath>
#include <queue>

#include <nlohmann/json.hpp>

namespace mu_path_planner {

using json = nlohmann::json;

namespace {

// ---------------------------------------------------------------------------
// Formula construction helpers
// ---------------------------------------------------------------------------

std::shared_ptr<Formula> make(Formula::Kind kind) {
    auto f = std::make_shared<Formula>();
    f->kind = kind;
    return f;
}

FormulaPtr make_atom(std::string name) {
    if (name == "true")  return make(Formula::Kind::True);
    if (name == "false") return make(Formula::Kind::False);
    auto f = std::make_shared<Formula>();
    f->kind = Formula::Kind::Atom;
    f->atom = std::move(name);
    return f;
}

// Thrown while walking the JSON, caught at the entry points. Parsing a
// snapshot is all-or-nothing: a half-read model is worse than a rejected one.
struct ParseError {
    std::string what;
};

[[noreturn]] void fail(const std::string& what) { throw ParseError{what}; }

std::optional<Formula::Modality> modality_from_name(const std::string& name) {
    if (name == "box")     return Formula::Modality::Box;
    if (name == "diamond") return Formula::Modality::Diamond;
    if (name == "Kw.box")  return Formula::Modality::KnowWhether;
    if (name == "C.box")   return Formula::Modality::CommonBox;
    if (name == "D.box")   return Formula::Modality::DistributedBox;
    return std::nullopt;
}

FormulaPtr build_formula(const json& node) {
    if (node.is_string()) return make_atom(node.get<std::string>());

    if (node.is_boolean())
        return make(node.get<bool>() ? Formula::Kind::True : Formula::Kind::False);

    if (!node.is_object())
        fail("a formula is a string, a boolean or an object");

    // plank wraps a goal or a precondition in {"formula": ...}
    if (node.contains("formula") && !node.contains("connective") &&
        !node.contains("modality-name"))
    {
        return build_formula(node.at("formula"));
    }

    if (node.contains("connective")) {
        const std::string connective = node.at("connective").get<std::string>();

        if (connective == "not") {
            if (!node.contains("formula")) fail("not without a formula");
            auto f = make(Formula::Kind::Not);
            f->subs.push_back(build_formula(node.at("formula")));
            return f;
        }

        Formula::Kind kind;
        if      (connective == "and")   kind = Formula::Kind::And;
        else if (connective == "or")    kind = Formula::Kind::Or;
        else if (connective == "imply") kind = Formula::Kind::Imply;
        else fail("unsupported connective: " + connective);

        if (!node.contains("formulas") || !node.at("formulas").is_array())
            fail(connective + " without a formulas array");

        auto f = std::make_shared<Formula>();
        f->kind = kind;
        for (const auto& sub : node.at("formulas"))
            f->subs.push_back(build_formula(sub));

        if (f->subs.empty()) fail(connective + " over no formulas");
        if (kind == Formula::Kind::Imply && f->subs.size() != 2)
            fail("imply takes exactly two formulas");
        return f;
    }

    if (node.contains("modality-name")) {
        const std::string name = node.at("modality-name").get<std::string>();
        const auto modality = modality_from_name(name);
        if (!modality) fail("unsupported modality: " + name);

        auto f = std::make_shared<Formula>();
        f->kind = Formula::Kind::Modal;
        f->modality = *modality;

        if (!node.contains("modality-index") || !node.at("modality-index").is_array())
            fail(name + " without a modality-index array");
        for (const auto& agent : node.at("modality-index"))
            f->index.push_back(agent.get<std::string>());
        if (f->index.empty()) fail(name + " over no agents");

        if (!node.contains("formula")) fail(name + " without a formula");
        f->subs.push_back(build_formula(node.at("formula")));
        return f;
    }

    fail("a formula object needs a connective, a modality-name or a formula");
}

// ---------------------------------------------------------------------------
// Snapshot parsing helpers
// ---------------------------------------------------------------------------

std::unordered_set<CellIdx> parse_cell_list(const json& node,
                                            const GridInfo& grid,
                                            const std::string& where)
{
    std::unordered_set<CellIdx> cells;
    const size_t n = static_cast<size_t>(grid.width) * grid.height;

    for (const auto& value : node) {
        const auto idx = value.get<int64_t>();
        if (idx < 0 || static_cast<size_t>(idx) >= n)
            fail(where + ": cell " + std::to_string(idx) + " is off the map");
        cells.insert(static_cast<CellIdx>(idx));
    }
    return cells;
}

std::unordered_set<CellIdx> parse_extent(const json& node,
                                         const GridInfo& grid,
                                         const std::string& where)
{
    if (!node.is_object())
        fail(where + ": an extent is an object");

    if (node.contains("cells")) {
        if (!node.at("cells").is_array()) fail(where + ": cells is an array");
        return parse_cell_list(node.at("cells"), grid, where);
    }

    if (node.contains("rect")) {
        const auto& rect = node.at("rect");
        const auto row    = rect.value("row", -1);
        const auto col    = rect.value("col", -1);
        const auto height = rect.value("height", 0);
        const auto width  = rect.value("width", 0);
        if (row < 0 || col < 0 || height <= 0 || width <= 0)
            fail(where + ": rect needs row, col, height and width");

        std::unordered_set<CellIdx> cells;
        for (int r = row; r < row + height; ++r) {
            for (int c = col; c < col + width; ++c) {
                if (r < 0 || c < 0 ||
                    static_cast<uint32_t>(r) >= grid.height ||
                    static_cast<uint32_t>(c) >= grid.width)
                {
                    continue;   // a rect may hang over the edge of the map
                }
                cells.insert(static_cast<CellIdx>(r) * grid.width +
                             static_cast<CellIdx>(c));
            }
        }
        return cells;
    }

    if (node.contains("bounds")) {
        const auto& bounds = node.at("bounds");
        for (const char* key : {"min_x", "min_y", "max_x", "max_y"})
            if (!bounds.contains(key)) fail(where + ": bounds needs " + key);

        const double min_x = bounds.at("min_x").get<double>();
        const double min_y = bounds.at("min_y").get<double>();
        const double max_x = bounds.at("max_x").get<double>();
        const double max_y = bounds.at("max_y").get<double>();

        std::unordered_set<CellIdx> cells;
        for (uint32_t r = 0; r < grid.height; ++r) {
            for (uint32_t c = 0; c < grid.width; ++c) {
                const double x = grid.origin_x + (c + 0.5) * grid.resolution;
                const double y = grid.origin_y + (r + 0.5) * grid.resolution;
                if (x >= min_x && x <= max_x && y >= min_y && y <= max_y)
                    cells.insert(r * grid.width + c);
            }
        }
        return cells;
    }

    fail(where + ": an extent is given by cells, rect or bounds");
}

// ---------------------------------------------------------------------------
// Map atoms
// ---------------------------------------------------------------------------

CellState state_of(CellIdx cell, const EvalContext& ctx) {
    if (ctx.cell_state && cell < ctx.cell_state->size())
        return (*ctx.cell_state)[cell];

    if (ctx.graph && cell < ctx.graph->obstacle.size())
        return ctx.graph->obstacle[cell] ? CellState::Occupied : CellState::Free;

    return CellState::Unknown;
}

}  // namespace

// ---------------------------------------------------------------------------
// GridInfo
// ---------------------------------------------------------------------------

std::optional<CellIdx> GridInfo::cell_at(double x, double y) const {
    if (width == 0 || height == 0 || resolution <= 0.0) return std::nullopt;

    // A pose sitting on a cell boundary must not be placed by the last bit of
    // the number it was divided by. The resolution of an OccupancyGrid travels
    // as a float, so x = 2.0 m over 0.1 m/cell arrives here as 19.9999997
    // cells and floors to 19, while the same map built with a double gives
    // exactly 20: the robot would resolve one cell away from itself depending
    // on which side of the wire the map came from. A quotient within a
    // millionth of an integer is taken to be that integer.
    // The tolerance is relative, because the error is: a float32 resolution
    // is wrong in the seventh significant digit, so the quotient it produces
    // is off by that much *of the quotient*. A fixed tolerance snaps y = 1.6 m
    // and misses y = 12.4 m, which is the same bug one aisle further north.
    // It stays far below half a cell either way, so a pose genuinely inside a
    // cell is never moved out of it.
    constexpr double kRelativeSlack = 1e-6;
    auto snapped = [](double quotient) {
        const double nearest = std::round(quotient);
        const double slack = kRelativeSlack * std::max(1.0, std::abs(quotient));
        return std::abs(quotient - nearest) < slack ? nearest : quotient;
    };

    const double fc = std::floor(snapped((x - origin_x) / resolution));
    const double fr = std::floor(snapped((y - origin_y) / resolution));
    if (fc < 0.0 || fr < 0.0) return std::nullopt;
    if (fc >= static_cast<double>(width) || fr >= static_cast<double>(height))
        return std::nullopt;

    return static_cast<CellIdx>(fr) * width + static_cast<CellIdx>(fc);
}

// ---------------------------------------------------------------------------
// Content hashing
// ---------------------------------------------------------------------------

uint64_t content_hash(const void* data, size_t bytes) {
    // FNV-1a, 64 bit: the offset basis and the prime are the published ones.
    uint64_t hash = 1469598103934665603ull;
    const auto* bytes_in = static_cast<const unsigned char*>(data);
    for (size_t i = 0; i < bytes; ++i) {
        hash ^= bytes_in[i];
        hash *= 1099511628211ull;
    }
    return hash;
}

uint64_t content_hash(const std::string& text) {
    return content_hash(text.data(), text.size());
}

// ---------------------------------------------------------------------------
// ZoneGrounding
// ---------------------------------------------------------------------------

const std::unordered_set<CellIdx>* ZoneGrounding::cells_in(
    const std::string& world) const
{
    const auto it = per_world.find(world);
    if (it != per_world.end()) return &it->second;
    if (global) return &*global;
    return nullptr;
}

// ---------------------------------------------------------------------------
// EpistemicSnapshot
// ---------------------------------------------------------------------------

std::vector<std::string> EpistemicSnapshot::accessible(
    const std::string& agent, const std::string& world) const
{
    const auto agent_it = relations.find(agent);
    if (agent_it == relations.end()) return {world};

    const auto world_it = agent_it->second.find(world);
    if (world_it == agent_it->second.end()) return {world};

    return world_it->second;
}

std::vector<std::string> EpistemicSnapshot::common_accessible(
    const std::vector<std::string>& group, const std::string& world) const
{
    std::vector<std::string> reached;
    std::unordered_set<std::string> seen;
    std::queue<std::string> frontier;

    frontier.push(world);
    seen.insert(world);

    while (!frontier.empty()) {
        const std::string current = frontier.front();
        frontier.pop();
        reached.push_back(current);

        for (const auto& agent : group) {
            for (const auto& next : accessible(agent, current)) {
                if (seen.insert(next).second) frontier.push(next);
            }
        }
    }

    // The starting world is only in the closure if some agent's relation leads
    // back to it, which under S5 it always does; dropping it here would make
    // common knowledge weaker than knowledge.
    return reached;
}

// ---------------------------------------------------------------------------
// Parsing
// ---------------------------------------------------------------------------

FormulaPtr parse_formula(const std::string& json_text, std::string& error) {
    error.clear();
    try {
        return build_formula(json::parse(json_text));
    } catch (const ParseError& e) {
        error = e.what;
    } catch (const json::exception& e) {
        error = std::string("malformed JSON: ") + e.what();
    }
    return nullptr;
}

EpistemicSnapshot parse_snapshot(const std::string& json_text,
                                 const GridInfo& grid)
{
    EpistemicSnapshot snapshot;

    try {
        const json root = json::parse(json_text);
        if (!root.is_object()) fail("a snapshot is an object");

        // Worlds -------------------------------------------------------
        if (!root.contains("worlds") || !root.at("worlds").is_array())
            fail("a snapshot needs a worlds array");
        for (const auto& world : root.at("worlds"))
            snapshot.worlds.push_back(world.get<std::string>());
        if (snapshot.worlds.empty()) fail("a snapshot needs at least one world");

        const std::unordered_set<std::string> known_worlds(
            snapshot.worlds.begin(), snapshot.worlds.end());

        // Designated ---------------------------------------------------
        if (!root.contains("designated") || !root.at("designated").is_array())
            fail("a snapshot needs a designated array");
        for (const auto& world : root.at("designated")) {
            const auto name = world.get<std::string>();
            if (!known_worlds.count(name))
                fail("designated world " + name + " is not among the worlds");
            snapshot.designated.push_back(name);
        }
        if (snapshot.designated.empty())
            fail("a snapshot needs at least one designated world");

        // Relations ----------------------------------------------------
        if (root.contains("relations")) {
            for (const auto& [agent, per_world] : root.at("relations").items()) {
                for (const auto& [world, targets] : per_world.items()) {
                    if (!known_worlds.count(world))
                        fail("relation of " + agent + " leaves from unknown world " + world);
                    std::vector<std::string> reachable;
                    for (const auto& target : targets) {
                        const auto name = target.get<std::string>();
                        if (!known_worlds.count(name))
                            fail("relation of " + agent + " reaches unknown world " + name);
                        reachable.push_back(name);
                    }
                    snapshot.relations[agent][world] = std::move(reachable);
                }
            }
        }

        // Labels -------------------------------------------------------
        if (root.contains("labels")) {
            for (const auto& [world, atoms] : root.at("labels").items()) {
                if (!known_worlds.count(world))
                    fail("labels for unknown world " + world);
                for (const auto& atom : atoms)
                    snapshot.labels[world].insert(atom.get<std::string>());
            }
        }

        // Agents -------------------------------------------------------
        if (root.contains("agents")) {
            for (const auto& [id_text, entry] : root.at("agents").items()) {
                uint32_t id = 0;
                try {
                    id = static_cast<uint32_t>(std::stoul(id_text));
                } catch (const std::exception&) {
                    fail("agent key " + id_text + " is not a numeric id");
                }

                AgentInfo info;
                info.name = entry.value("name", id_text);

                if (entry.contains("cell")) {
                    const auto idx = entry.at("cell").get<int64_t>();
                    const size_t n = static_cast<size_t>(grid.width) * grid.height;
                    if (idx < 0 || static_cast<size_t>(idx) >= n)
                        fail("agent " + info.name + " is off the map");
                    info.cell = static_cast<CellIdx>(idx);
                } else if (entry.contains("pose")) {
                    const auto& pose = entry.at("pose");
                    if (!pose.contains("x") || !pose.contains("y"))
                        fail("agent " + info.name + " has a pose without x and y");
                    info.cell = grid.cell_at(pose.at("x").get<double>(),
                                             pose.at("y").get<double>());
                    if (!info.cell)
                        fail("agent " + info.name + " is outside the map");
                }

                snapshot.agents[id] = std::move(info);
            }
        }

        // Zones --------------------------------------------------------
        if (root.contains("zones")) {
            for (const auto& [zone, extent] : root.at("zones").items()) {
                const std::string where = "zone " + zone;
                if (!extent.is_object()) fail(where + ": a zone is an object");

                ZoneGrounding grounding;
                if (extent.contains("worlds")) {
                    for (const auto& [world, per] : extent.at("worlds").items()) {
                        if (!known_worlds.count(world))
                            fail(where + ": unknown world " + world);
                        grounding.per_world[world] =
                            parse_extent(per, grid, where + " in " + world);
                    }
                } else {
                    grounding.global = parse_extent(extent, grid, where);
                }
                snapshot.zones[zone] = std::move(grounding);
            }
        }

        snapshot.ok = true;
    } catch (const ParseError& e) {
        snapshot = EpistemicSnapshot{};
        snapshot.error = e.what;
    } catch (const json::exception& e) {
        snapshot = EpistemicSnapshot{};
        snapshot.error = std::string("malformed JSON: ") + e.what();
    }

    return snapshot;
}

// ---------------------------------------------------------------------------
// Evaluation
// ---------------------------------------------------------------------------

bool holds(const Formula& f, CellIdx cell, const std::string& world,
           const EvalContext& ctx)
{
    switch (f.kind) {
        case Formula::Kind::True:  return true;
        case Formula::Kind::False: return false;

        case Formula::Kind::Atom: {
            // The map answers before the model does: obstacle, free and
            // unknown are about the cell in front of the robot, not about
            // anything a world could disagree over.
            if (f.atom == "obstacle") return state_of(cell, ctx) != CellState::Free;
            if (f.atom == "free")     return state_of(cell, ctx) == CellState::Free;
            if (f.atom == "unknown")  return state_of(cell, ctx) == CellState::Unknown;

            if (!ctx.snapshot) return false;

            const auto zone = ctx.snapshot->zones.find(f.atom);
            if (zone != ctx.snapshot->zones.end()) {
                const auto* cells = zone->second.cells_in(world);
                return cells && cells->count(cell) > 0;
            }

            const auto labels = ctx.snapshot->labels.find(world);
            return labels != ctx.snapshot->labels.end() &&
                   labels->second.count(f.atom) > 0;
        }

        case Formula::Kind::Not:
            return !holds(*f.subs.front(), cell, world, ctx);

        case Formula::Kind::And:
            for (const auto& sub : f.subs)
                if (!holds(*sub, cell, world, ctx)) return false;
            return true;

        case Formula::Kind::Or:
            for (const auto& sub : f.subs)
                if (holds(*sub, cell, world, ctx)) return true;
            return false;

        case Formula::Kind::Imply:
            return !holds(*f.subs.front(), cell, world, ctx) ||
                   holds(*f.subs.back(), cell, world, ctx);

        case Formula::Kind::Modal: {
            if (!ctx.snapshot) return false;
            const Formula& sub = *f.subs.front();

            // Which worlds the modality quantifies over. Box and diamond over
            // several agents read as everybody-knows and its dual; common and
            // distributed knowledge combine the relations instead.
            std::vector<std::string> range;
            switch (f.modality) {
                case Formula::Modality::CommonBox:
                    range = ctx.snapshot->common_accessible(f.index, world);
                    break;

                case Formula::Modality::DistributedBox: {
                    // The intersection of the relations: what the group could
                    // tell apart by pooling what each of them knows.
                    std::vector<std::string> pooled =
                        ctx.snapshot->accessible(f.index.front(), world);
                    for (size_t i = 1; i < f.index.size(); ++i) {
                        const auto next = ctx.snapshot->accessible(f.index[i], world);
                        const std::unordered_set<std::string> keep(next.begin(), next.end());
                        pooled.erase(
                            std::remove_if(pooled.begin(), pooled.end(),
                                           [&keep](const std::string& w) {
                                               return keep.count(w) == 0;
                                           }),
                            pooled.end());
                    }
                    range = std::move(pooled);
                    break;
                }

                default:
                    for (const auto& agent : f.index) {
                        for (const auto& w : ctx.snapshot->accessible(agent, world))
                            range.push_back(w);
                    }
                    break;
            }

            if (f.modality == Formula::Modality::Diamond) {
                for (const auto& w : range)
                    if (holds(sub, cell, w, ctx)) return true;
                return false;
            }

            if (f.modality == Formula::Modality::KnowWhether) {
                bool all_true = true;
                bool all_false = true;
                for (const auto& w : range) {
                    if (holds(sub, cell, w, ctx)) all_false = false;
                    else                          all_true = false;
                }
                return all_true || all_false;
            }

            for (const auto& w : range)
                if (!holds(sub, cell, w, ctx)) return false;
            return true;
        }
    }

    return false;
}

bool holds_at_designated(const Formula& f, CellIdx cell, const EvalContext& ctx)
{
    if (!ctx.snapshot) return false;
    for (const auto& world : ctx.snapshot->designated)
        if (!holds(f, cell, world, ctx)) return false;
    return !ctx.snapshot->designated.empty();
}

// ---------------------------------------------------------------------------
// Resolving a query
// ---------------------------------------------------------------------------

namespace {

/// The free cells within @p range steps of any cell of @p seeds, measured over
/// the grid rather than over free space: a sensor reads across a cell the
/// robot cannot drive through, which is exactly the case that matters.
std::unordered_set<CellIdx> observable_from(
    const std::unordered_set<CellIdx>& seeds,
    uint32_t range,
    const OccupancyGraph& graph,
    const std::vector<CellState>& cell_state)
{
    const size_t n = static_cast<size_t>(graph.width) * graph.height;

    const auto is_free = [&](CellIdx cell) {
        if (!cell_state.empty() && cell < cell_state.size())
            return cell_state[cell] == CellState::Free;
        return cell < graph.obstacle.size() && !graph.obstacle[cell];
    };

    std::unordered_set<CellIdx> observers;
    std::vector<bool> seen(n, false);
    std::queue<std::pair<CellIdx, uint32_t>> frontier;

    for (CellIdx seed : seeds) {
        if (seed >= n || seen[seed]) continue;
        seen[seed] = true;
        frontier.emplace(seed, 0u);
    }

    const int dr[] = {-1, 1, 0, 0};
    const int dc[] = { 0, 0,-1, 1};

    while (!frontier.empty()) {
        const auto [cell, depth] = frontier.front();
        frontier.pop();

        if (is_free(cell)) observers.insert(cell);
        if (depth == range) continue;

        const int r = static_cast<int>(cell / graph.width);
        const int c = static_cast<int>(cell % graph.width);
        for (int d = 0; d < 4; ++d) {
            const int nr = r + dr[d];
            const int nc = c + dc[d];
            if (!graph.in_bounds(nr, nc)) continue;
            const CellIdx next = static_cast<CellIdx>(nr) * graph.width +
                                 static_cast<CellIdx>(nc);
            if (seen[next]) continue;
            seen[next] = true;
            frontier.emplace(next, depth + 1);
        }
    }

    return observers;
}

}  // namespace

ResolvedQuery resolve_query(const EpistemicSnapshot& snapshot,
                            const OccupancyGraph& graph,
                            const std::vector<CellState>& cell_state,
                            const QuerySpec& spec)
{
    ResolvedQuery out;

    if (!snapshot.ok) {
        out.error = "no usable epistemic state: " + snapshot.error;
        return out;
    }

    // The agent, and where it stands ------------------------------------
    const auto agent_it = snapshot.agents.find(spec.agent_id);
    if (agent_it == snapshot.agents.end()) {
        out.error = "the epistemic state does not mention agent " +
                    std::to_string(spec.agent_id);
        return out;
    }
    const AgentInfo& agent = agent_it->second;
    if (!agent.cell) {
        out.error = "the epistemic state does not locate agent " + agent.name;
        return out;
    }
    out.start = *agent.cell;

    // The goal zone ------------------------------------------------------
    if (spec.goal_zone.empty()) {
        out.error = "the query names no goal zone";
        return out;
    }
    const auto zone_it = snapshot.zones.find(spec.goal_zone);
    if (zone_it == snapshot.zones.end()) {
        out.error = "the epistemic state grounds no zone named " + spec.goal_zone;
        return out;
    }
    const ZoneGrounding& zone = zone_it->second;

    // The safety constraint ----------------------------------------------
    FormulaPtr safety;
    if (spec.safety_formula_json.empty()) {
        // ¬obstacle, which is the constraint the planner is named after.
        auto negation = std::make_shared<Formula>();
        negation->kind = Formula::Kind::Not;
        negation->subs.push_back(make_atom("obstacle"));
        safety = negation;
    } else {
        std::string error;
        safety = parse_formula(spec.safety_formula_json, error);
        if (!safety) {
            out.error = "safety formula: " + error;
            return out;
        }
    }

    EvalContext ctx;
    ctx.snapshot   = &snapshot;
    ctx.graph      = &graph;
    ctx.cell_state = cell_state.empty() ? nullptr : &cell_state;

    const size_t n = static_cast<size_t>(graph.width) * graph.height;

    const auto is_free = [&](CellIdx cell) {
        if (!cell_state.empty() && cell < cell_state.size())
            return cell_state[cell] == CellState::Free;
        return cell < graph.obstacle.size() && !graph.obstacle[cell];
    };

    for (CellIdx cell = 0; cell < static_cast<CellIdx>(n); ++cell) {
        if (!is_free(cell)) continue;               // unknown is not free
        if (holds_at_designated(*safety, cell, ctx)) out.safe.insert(cell);
    }

    // Where the goal in fact is: the cells every designated world agrees on.
    bool first = true;
    for (const auto& world : snapshot.designated) {
        const auto* cells = zone.cells_in(world);
        if (!cells) { out.goal.clear(); break; }

        if (first) {
            for (CellIdx cell : *cells)
                if (is_free(cell)) out.goal.insert(cell);
            first = false;
        } else {
            for (auto it = out.goal.begin(); it != out.goal.end(); ) {
                it = cells->count(*it) ? std::next(it) : out.goal.erase(it);
            }
        }
    }

    // What the agent can tell about it: the worlds it cannot rule out from
    // any world it takes to be the case.
    std::vector<std::string> horizon;
    {
        std::unordered_set<std::string> seen;
        for (const auto& world : snapshot.designated) {
            for (const auto& reachable : snapshot.accessible(agent.name, world))
                if (seen.insert(reachable).second) horizon.push_back(reachable);
        }
    }

    std::unordered_set<CellIdx> in_every;   // goal cell however things turn out
    std::unordered_set<CellIdx> in_some;
    first = true;
    for (const auto& world : horizon) {
        const auto* cells = zone.cells_in(world);
        const std::unordered_set<CellIdx> empty;
        const auto& here = cells ? *cells : empty;

        for (CellIdx cell : here) in_some.insert(cell);

        if (first) {
            in_every = here;
            first = false;
        } else {
            for (auto it = in_every.begin(); it != in_every.end(); ) {
                it = here.count(*it) ? std::next(it) : in_every.erase(it);
            }
        }
    }

    // K_i(at_goal) holds where the cell is a goal cell and no world the agent
    // holds possible says otherwise.
    for (CellIdx cell : out.goal)
        if (in_every.count(cell)) out.known_goal.insert(cell);

    // The cells the agent cannot decide: in the zone in some world it holds
    // possible, out of it in another. Resolving one is what sensing is for.
    for (CellIdx cell : in_some)
        if (!in_every.count(cell)) out.disputed.insert(cell);

    if (spec.require_epistemic_goal) {
        out.sensing = observable_from(out.disputed, spec.sensor_range_cells,
                                      graph, cell_state);
        for (CellIdx cell : out.known_goal) out.sensing.insert(cell);
    }

    out.ok = true;
    return out;
}

}  // namespace mu_path_planner
