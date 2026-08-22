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

#include <gtest/gtest.h>

#include "mu_path_planner/epistemic_state.hpp"

using namespace mu_path_planner;

namespace {

constexpr uint32_t kSize = 5;

GridInfo make_grid() {
    GridInfo grid;
    grid.width      = kSize;
    grid.height     = kSize;
    grid.resolution = 1.0;
    grid.origin_x   = 0.0;
    grid.origin_y   = 0.0;
    return grid;
}

OccupancyGraph make_free_graph() {
    OccupancyGraph g;
    g.width  = kSize;
    g.height = kSize;
    g.obstacle.assign(static_cast<size_t>(kSize) * kSize, false);
    g.build_adjacency();
    return g;
}

CellIdx cell(uint32_t row, uint32_t col) { return row * kSize + col; }

// A snapshot where the agent cannot tell w0 from w1, and the two worlds
// disagree about whether cell (2,4) belongs to the bay. That disagreement is
// the whole subject: it is what a sensing action is there to settle.
std::string uncertain_snapshot() {
    return R"({
      "worlds": ["w0", "w1"],
      "designated": ["w0"],
      "relations": { "r1": { "w0": ["w0","w1"], "w1": ["w0","w1"] } },
      "labels": { "w0": ["link_up"], "w1": [] },
      "agents": { "1": { "name": "r1", "cell": 10 } },
      "zones": {
        "bay": { "worlds": { "w0": { "cells": [14] }, "w1": { "cells": [] } } }
      }
    })";
}

// The same map, with the bay grounded once for every world: nothing left to
// find out, so the agent already knows a bay cell when it stands on one.
std::string settled_snapshot() {
    return R"({
      "worlds": ["w0", "w1"],
      "designated": ["w0"],
      "relations": { "r1": { "w0": ["w0","w1"], "w1": ["w0","w1"] } },
      "labels": { "w0": [], "w1": [] },
      "agents": { "1": { "name": "r1", "cell": 10 } },
      "zones": { "bay": { "cells": [14] } }
    })";
}

}  // namespace

// ---------------------------------------------------------------------------
// Snapshot parsing
// ---------------------------------------------------------------------------

TEST(Snapshot, ParsesWorldsRelationsAndZones) {
    const auto snapshot = parse_snapshot(uncertain_snapshot(), make_grid());
    ASSERT_TRUE(snapshot.ok) << snapshot.error;

    EXPECT_EQ(snapshot.worlds.size(), 2u);
    ASSERT_EQ(snapshot.designated.size(), 1u);
    EXPECT_EQ(snapshot.designated.front(), "w0");
    EXPECT_EQ(snapshot.accessible("r1", "w0").size(), 2u);
    EXPECT_EQ(snapshot.labels.at("w0").count("link_up"), 1u);

    ASSERT_EQ(snapshot.agents.count(1u), 1u);
    EXPECT_EQ(snapshot.agents.at(1u).name, "r1");
    ASSERT_TRUE(snapshot.agents.at(1u).cell.has_value());
    EXPECT_EQ(*snapshot.agents.at(1u).cell, cell(2, 0));

    const auto& bay = snapshot.zones.at("bay");
    ASSERT_NE(bay.cells_in("w0"), nullptr);
    EXPECT_EQ(bay.cells_in("w0")->count(cell(2, 4)), 1u);
    EXPECT_TRUE(bay.cells_in("w1")->empty());
}

TEST(Snapshot, AnAgentWithoutARelationIsReflexive) {
    const auto snapshot = parse_snapshot(settled_snapshot(), make_grid());
    ASSERT_TRUE(snapshot.ok) << snapshot.error;

    const auto worlds = snapshot.accessible("nobody", "w0");
    ASSERT_EQ(worlds.size(), 1u);
    EXPECT_EQ(worlds.front(), "w0");
}

TEST(Snapshot, ResolvesAPoseAndMetricBounds) {
    const std::string text = R"({
      "worlds": ["w0"], "designated": ["w0"],
      "agents": { "7": { "name": "r7", "pose": { "x": 3.5, "y": 1.5 } } },
      "zones": {
        "dock": { "bounds": { "min_x": 3.0, "min_y": 0.0,
                              "max_x": 5.0, "max_y": 1.0 } },
        "depot": { "rect": { "row": 0, "col": 0, "height": 2, "width": 2 } }
      }
    })";

    const auto snapshot = parse_snapshot(text, make_grid());
    ASSERT_TRUE(snapshot.ok) << snapshot.error;

    // One metre cells with the origin at zero: (3.5, 1.5) is row 1, column 3.
    EXPECT_EQ(*snapshot.agents.at(7u).cell, cell(1, 3));

    const auto* dock = snapshot.zones.at("dock").cells_in("w0");
    ASSERT_NE(dock, nullptr);
    EXPECT_EQ(dock->count(cell(0, 3)), 1u);
    EXPECT_EQ(dock->count(cell(0, 4)), 1u);
    EXPECT_EQ(dock->count(cell(1, 3)), 0u);   // y = 1.5 is outside max_y

    const auto* depot = snapshot.zones.at("depot").cells_in("w0");
    ASSERT_NE(depot, nullptr);
    EXPECT_EQ(depot->size(), 4u);
}

TEST(Snapshot, MalformedInputIsRejectedRatherThanHalfRead) {
    for (const char* text : {
             "{ not json",
             R"({"designated": ["w0"]})",                       // no worlds
             R"({"worlds": ["w0"], "designated": []})",         // nothing designated
             R"({"worlds": ["w0"], "designated": ["w9"]})",     // unknown world
             R"({"worlds":["w0"],"designated":["w0"],
                 "zones": { "bay": { "cells": [999] } }})",     // off the map
             R"({"worlds":["w0"],"designated":["w0"],
                 "agents": { "r1": { "cell": 0 } }})",          // key is not an id
         })
    {
        const auto snapshot = parse_snapshot(text, make_grid());
        EXPECT_FALSE(snapshot.ok) << text;
        EXPECT_FALSE(snapshot.error.empty()) << text;
        EXPECT_TRUE(snapshot.worlds.empty()) << text;
    }
}

// ---------------------------------------------------------------------------
// Formulas
// ---------------------------------------------------------------------------

TEST(Formula, ParsesTheShapesPlankEmits) {
    std::string error;

    EXPECT_NE(parse_formula(R"("free")", error), nullptr) << error;
    EXPECT_NE(parse_formula(R"({"connective":"not","formula":"obstacle"})", error), nullptr) << error;
    EXPECT_NE(parse_formula(R"({"connective":"and","formulas":["free","link_up"]})", error), nullptr) << error;
    EXPECT_NE(parse_formula(R"({"connective":"imply","formulas":["free","link_up"]})", error), nullptr) << error;
    EXPECT_NE(parse_formula(R"({"modality-name":"box","modality-index":["r1"],"formula":"bay"})", error), nullptr) << error;

    // A goal or a precondition arrives wrapped, and unwraps.
    EXPECT_NE(parse_formula(R"({"formula":{"connective":"not","formula":"obstacle"}})", error), nullptr) << error;

    EXPECT_EQ(parse_formula("", error), nullptr);
    EXPECT_EQ(parse_formula(R"({"connective":"xor","formulas":["a","b"]})", error), nullptr);
    EXPECT_EQ(parse_formula(R"({"connective":"imply","formulas":["a"]})", error), nullptr);
    EXPECT_EQ(parse_formula(R"({"modality-name":"box","formula":"a"})", error), nullptr);
    EXPECT_EQ(parse_formula(R"({"modality-name":"nabla","modality-index":["r1"],"formula":"a"})", error), nullptr);
}

TEST(Formula, TheMapAtomsAnswerBeforeTheModelDoes) {
    const auto graph = make_free_graph();
    const auto snapshot = parse_snapshot(settled_snapshot(), make_grid());
    ASSERT_TRUE(snapshot.ok) << snapshot.error;

    std::vector<CellState> state(static_cast<size_t>(kSize) * kSize, CellState::Free);
    state[cell(1, 1)] = CellState::Occupied;
    state[cell(3, 3)] = CellState::Unknown;

    EvalContext ctx;
    ctx.snapshot   = &snapshot;
    ctx.graph      = &graph;
    ctx.cell_state = &state;

    std::string error;
    const auto free_here = parse_formula(R"("free")", error);
    const auto unknown_here = parse_formula(R"("unknown")", error);
    const auto obstacle_here = parse_formula(R"("obstacle")", error);
    ASSERT_NE(free_here, nullptr);

    EXPECT_TRUE(holds(*free_here, cell(0, 0), "w0", ctx));
    EXPECT_FALSE(holds(*free_here, cell(1, 1), "w0", ctx));
    EXPECT_TRUE(holds(*obstacle_here, cell(1, 1), "w0", ctx));

    // A cell nobody observed is an obstacle to the fixed point, and is also
    // reportable as unknown, which is what a frontier is made of.
    EXPECT_TRUE(holds(*obstacle_here, cell(3, 3), "w0", ctx));
    EXPECT_TRUE(holds(*unknown_here, cell(3, 3), "w0", ctx));
    EXPECT_FALSE(holds(*unknown_here, cell(1, 1), "w0", ctx));
}

TEST(Formula, ZoneAtomsAreCellIndexedAndLabelsAreNot) {
    const auto graph = make_free_graph();
    const auto snapshot = parse_snapshot(uncertain_snapshot(), make_grid());
    ASSERT_TRUE(snapshot.ok) << snapshot.error;

    EvalContext ctx;
    ctx.snapshot = &snapshot;
    ctx.graph    = &graph;

    std::string error;
    const auto in_bay  = parse_formula(R"("bay")", error);
    const auto link_up = parse_formula(R"("link_up")", error);

    EXPECT_TRUE(holds(*in_bay, cell(2, 4), "w0", ctx));
    EXPECT_FALSE(holds(*in_bay, cell(2, 4), "w1", ctx));
    EXPECT_FALSE(holds(*in_bay, cell(0, 0), "w0", ctx));

    // A label is a fact about the world, not about where the robot stands.
    EXPECT_TRUE(holds(*link_up, cell(0, 0), "w0", ctx));
    EXPECT_TRUE(holds(*link_up, cell(2, 4), "w0", ctx));
    EXPECT_FALSE(holds(*link_up, cell(0, 0), "w1", ctx));
}

TEST(Formula, KnowledgeQuantifiesOverTheWorldsAnAgentCannotRuleOut) {
    const auto graph = make_free_graph();
    const auto snapshot = parse_snapshot(uncertain_snapshot(), make_grid());
    ASSERT_TRUE(snapshot.ok) << snapshot.error;

    EvalContext ctx;
    ctx.snapshot = &snapshot;
    ctx.graph    = &graph;

    std::string error;
    const auto knows =
        parse_formula(R"({"modality-name":"box","modality-index":["r1"],"formula":"bay"})", error);
    const auto considers =
        parse_formula(R"({"modality-name":"diamond","modality-index":["r1"],"formula":"bay"})", error);
    const auto knows_whether =
        parse_formula(R"({"modality-name":"Kw.box","modality-index":["r1"],"formula":"bay"})", error);
    ASSERT_NE(knows, nullptr) << error;

    // The bay cell is a bay cell in w0 and not in w1, and r1 cannot tell them
    // apart: it holds the possibility without knowing.
    EXPECT_FALSE(holds(*knows, cell(2, 4), "w0", ctx));
    EXPECT_TRUE(holds(*considers, cell(2, 4), "w0", ctx));
    EXPECT_FALSE(holds(*knows_whether, cell(2, 4), "w0", ctx));

    // Where the worlds agree, knowing whether is settled.
    EXPECT_TRUE(holds(*knows_whether, cell(0, 0), "w0", ctx));
}

TEST(Formula, CommonKnowledgeFollowsTheChainAndDistributedPoolsIt) {
    // r1 cannot tell w0 from w1; r2 cannot tell w1 from w2. Neither of them
    // alone reaches w2 from w0, their common knowledge does, and what they
    // could tell apart together is only w1.
    const std::string text = R"({
      "worlds": ["w0","w1","w2"],
      "designated": ["w1"],
      "relations": {
        "r1": { "w0": ["w0","w1"], "w1": ["w0","w1"], "w2": ["w2"] },
        "r2": { "w0": ["w0"], "w1": ["w1","w2"], "w2": ["w1","w2"] }
      },
      "labels": { "w0": ["p"], "w1": ["p"], "w2": [] }
    })";

    const auto graph = make_free_graph();
    const auto snapshot = parse_snapshot(text, make_grid());
    ASSERT_TRUE(snapshot.ok) << snapshot.error;

    EvalContext ctx;
    ctx.snapshot = &snapshot;
    ctx.graph    = &graph;

    std::string error;
    const auto r1_knows =
        parse_formula(R"({"modality-name":"box","modality-index":["r1"],"formula":"p"})", error);
    const auto common =
        parse_formula(R"({"modality-name":"C.box","modality-index":["r1","r2"],"formula":"p"})", error);
    const auto distributed =
        parse_formula(R"({"modality-name":"D.box","modality-index":["r1","r2"],"formula":"p"})", error);
    ASSERT_NE(common, nullptr) << error;

    EXPECT_TRUE(holds(*r1_knows, cell(0, 0), "w0", ctx));
    EXPECT_FALSE(holds(*common, cell(0, 0), "w0", ctx));   // the chain reaches w2
    EXPECT_TRUE(holds(*distributed, cell(0, 0), "w1", ctx));
}

// ---------------------------------------------------------------------------
// Resolving a query
// ---------------------------------------------------------------------------

namespace {

QuerySpec bay_query(bool epistemic) {
    QuerySpec spec;
    spec.agent_id = 1;
    spec.goal_zone = "bay";
    spec.require_epistemic_goal = epistemic;
    spec.sensor_range_cells = 1;
    return spec;
}

}  // namespace

TEST(ResolveQuery, GroundsTheGoalZoneAndTheDefaultSafetyConstraint) {
    auto graph = make_free_graph();
    graph.obstacle[cell(1, 1)] = true;
    graph.build_adjacency();

    const auto snapshot = parse_snapshot(settled_snapshot(), make_grid());
    ASSERT_TRUE(snapshot.ok) << snapshot.error;

    const auto query = resolve_query(snapshot, graph, {}, bay_query(false));
    ASSERT_TRUE(query.ok) << query.error;

    EXPECT_EQ(query.start, cell(2, 0));
    EXPECT_EQ(query.goal, std::unordered_set<CellIdx>{cell(2, 4)});

    // The default constraint is ¬obstacle, so every free cell is safe and the
    // occupied one is not.
    EXPECT_EQ(query.safe.size(), static_cast<size_t>(kSize) * kSize - 1);
    EXPECT_EQ(query.safe.count(cell(1, 1)), 0u);
}

TEST(ResolveQuery, AnAgreedZoneIsAlreadyKnownAndNeedsNoSensing) {
    const auto graph = make_free_graph();
    const auto snapshot = parse_snapshot(settled_snapshot(), make_grid());
    ASSERT_TRUE(snapshot.ok) << snapshot.error;

    const auto query = resolve_query(snapshot, graph, {}, bay_query(true));
    ASSERT_TRUE(query.ok) << query.error;

    EXPECT_EQ(query.known_goal, query.goal);
    EXPECT_TRUE(query.disputed.empty());
    EXPECT_EQ(query.sensing, std::unordered_set<CellIdx>{cell(2, 4)});
}

TEST(ResolveQuery, ADisputedCellBecomesSomewhereToSenseFrom) {
    const auto graph = make_free_graph();
    const auto snapshot = parse_snapshot(uncertain_snapshot(), make_grid());
    ASSERT_TRUE(snapshot.ok) << snapshot.error;

    const auto query = resolve_query(snapshot, graph, {}, bay_query(true));
    ASSERT_TRUE(query.ok) << query.error;

    // It is a goal cell as things stand, and the agent does not know it.
    EXPECT_EQ(query.goal.count(cell(2, 4)), 1u);
    EXPECT_TRUE(query.known_goal.empty());
    EXPECT_EQ(query.disputed, std::unordered_set<CellIdx>{cell(2, 4)});

    // With a reach of one cell, the agent can settle it from the cell itself
    // or from any of its neighbours.
    EXPECT_EQ(query.sensing.count(cell(2, 4)), 1u);
    EXPECT_EQ(query.sensing.count(cell(1, 4)), 1u);
    EXPECT_EQ(query.sensing.count(cell(2, 3)), 1u);
    EXPECT_EQ(query.sensing.count(cell(0, 0)), 0u);
}

TEST(ResolveQuery, SensingReachesOverACellNoOneHasObserved) {
    // The disputed cell is the far end of a corridor nobody has looked down,
    // so it is not free and cannot be stood on. A sensor that reaches two
    // cells still resolves it, from the last free cell before it.
    auto graph = make_free_graph();
    std::vector<CellState> state(static_cast<size_t>(kSize) * kSize, CellState::Free);
    state[cell(2, 3)] = CellState::Unknown;
    state[cell(2, 4)] = CellState::Unknown;
    for (size_t i = 0; i < state.size(); ++i)
        graph.obstacle[i] = (state[i] != CellState::Free);
    graph.build_adjacency();

    const auto snapshot = parse_snapshot(uncertain_snapshot(), make_grid());
    ASSERT_TRUE(snapshot.ok) << snapshot.error;

    auto spec = bay_query(true);
    spec.sensor_range_cells = 2;

    const auto query = resolve_query(snapshot, graph, state, spec);
    ASSERT_TRUE(query.ok) << query.error;

    // Unknown is not free, so the cell is not a goal cell to drive to, but it
    // is still the cell the agent is unsure about.
    EXPECT_TRUE(query.goal.empty());
    EXPECT_EQ(query.disputed, std::unordered_set<CellIdx>{cell(2, 4)});

    EXPECT_EQ(query.sensing.count(cell(2, 4)), 0u);   // not free: cannot stand there
    EXPECT_EQ(query.sensing.count(cell(2, 2)), 1u);   // two cells away, and free
    EXPECT_EQ(query.sensing.count(cell(1, 4)), 1u);
}

TEST(ResolveQuery, ASafetyFormulaFromTheQueryNarrowsTheSafeSet) {
    const auto graph = make_free_graph();
    const auto snapshot = parse_snapshot(settled_snapshot(), make_grid());
    ASSERT_TRUE(snapshot.ok) << snapshot.error;

    auto spec = bay_query(false);
    // Free, and out of the bay: a constraint that keeps the robot off the
    // very cells it is heading for, which is worth being able to express.
    spec.safety_formula_json =
        R"({"connective":"and","formulas":["free",{"connective":"not","formula":"bay"}]})";

    const auto query = resolve_query(snapshot, graph, {}, spec);
    ASSERT_TRUE(query.ok) << query.error;

    EXPECT_EQ(query.safe.size(), static_cast<size_t>(kSize) * kSize - 1);
    EXPECT_EQ(query.safe.count(cell(2, 4)), 0u);
}

TEST(ResolveQuery, ABadSafetyFormulaIsRefusedRatherThanIgnored) {
    const auto graph = make_free_graph();
    const auto snapshot = parse_snapshot(settled_snapshot(), make_grid());

    auto spec = bay_query(false);
    spec.safety_formula_json = R"({"connective":"xor","formulas":["a","b"]})";

    const auto query = resolve_query(snapshot, graph, {}, spec);
    EXPECT_FALSE(query.ok);
    EXPECT_NE(query.error.find("safety formula"), std::string::npos);
}

TEST(ResolveQuery, RefusesWhatTheSnapshotDoesNotSay) {
    const auto graph = make_free_graph();
    const auto snapshot = parse_snapshot(settled_snapshot(), make_grid());

    auto unknown_agent = bay_query(false);
    unknown_agent.agent_id = 9;
    EXPECT_FALSE(resolve_query(snapshot, graph, {}, unknown_agent).ok);

    auto unknown_zone = bay_query(false);
    unknown_zone.goal_zone = "hangar";
    EXPECT_FALSE(resolve_query(snapshot, graph, {}, unknown_zone).ok);

    auto no_zone = bay_query(false);
    no_zone.goal_zone.clear();
    EXPECT_FALSE(resolve_query(snapshot, graph, {}, no_zone).ok);

    // An agent the model mentions without placing on the map is no better
    // than an agent it does not mention.
    const auto unplaced = parse_snapshot(R"({
      "worlds": ["w0"], "designated": ["w0"],
      "agents": { "1": { "name": "r1" } },
      "zones": { "bay": { "cells": [14] } }
    })", make_grid());
    ASSERT_TRUE(unplaced.ok) << unplaced.error;
    const auto query = resolve_query(unplaced, graph, {}, bay_query(false));
    EXPECT_FALSE(query.ok);
    EXPECT_NE(query.error.find("locate"), std::string::npos);
}

// ---------------------------------------------------------------------------
// The whole way through: snapshot to route
// ---------------------------------------------------------------------------

TEST(ResolveQuery, TheEpistemicRouteStopsWhereTheSensingIsWorthTaking) {
    auto graph = make_free_graph();
    std::vector<CellState> state(static_cast<size_t>(kSize) * kSize, CellState::Free);
    state[cell(2, 4)] = CellState::Unknown;
    for (size_t i = 0; i < state.size(); ++i)
        graph.obstacle[i] = (state[i] != CellState::Free);
    graph.build_adjacency();

    const auto snapshot = parse_snapshot(uncertain_snapshot(), make_grid());
    ASSERT_TRUE(snapshot.ok) << snapshot.error;

    const auto query = resolve_query(snapshot, graph, state, bay_query(true));
    ASSERT_TRUE(query.ok) << query.error;

    const auto plan = mu_reach_epistemic(graph, query.goal, query.safe,
                                         query.sensing, query.start);

    ASSERT_FALSE(plan.path.empty());
    EXPECT_EQ(plan.path.front(), query.start);

    // The route ends at a cell the agent can sense the disputed cell from,
    // not at the disputed cell, which it cannot drive into.
    EXPECT_EQ(query.sensing.count(plan.path.back()), 1u);
    EXPECT_NE(plan.path.back(), cell(2, 4));
    EXPECT_FALSE(plan.sensing_waypoints.empty());

    // Asked for the ontic goal instead, there is nowhere to go: the only cell
    // of the bay is one nobody has observed.
    const auto ontic = resolve_query(snapshot, graph, state, bay_query(false));
    ASSERT_TRUE(ontic.ok) << ontic.error;
    EXPECT_TRUE(ontic.goal.empty());
    EXPECT_TRUE(mu_reach(graph, ontic.goal, ontic.safe, ontic.start).path.empty());
}
