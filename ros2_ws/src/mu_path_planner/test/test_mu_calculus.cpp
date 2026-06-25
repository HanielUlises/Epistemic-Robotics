#include <gtest/gtest.h>
#include "mu_path_planner/mu_calculus.hpp"

using namespace mu_path_planner;

// Helper: build a simple W×H grid with no obstacles
OccupancyGraph make_free_grid(uint32_t W, uint32_t H) {
    OccupancyGraph g;
    g.width  = W;
    g.height = H;
    g.obstacle.assign(static_cast<size_t>(W) * H, false);
    g.build_adjacency();
    return g;
}

// Helper: build a grid with a wall column at col=wall_col
OccupancyGraph make_walled_grid(uint32_t W, uint32_t H, uint32_t wall_col) {
    OccupancyGraph g;
    g.width  = W;
    g.height = H;
    g.obstacle.assign(static_cast<size_t>(W) * H, false);
    for (uint32_t r = 0; r < H; ++r)
        g.obstacle[g.cell(r, wall_col)] = true;
    g.build_adjacency();
    return g;
}

// ---------------------------------------------------------------------------
// Test 1: free grid — start can reach goal
// ---------------------------------------------------------------------------
TEST(MuCalculus, FreeGridReachable) {
    auto g = make_free_grid(5, 5);

    std::unordered_set<CellIdx> goal  = { g.cell(0, 4) };
    std::unordered_set<CellIdx> safe;
    for (CellIdx i = 0; i < 25; ++i) safe.insert(i);

    CellIdx start = g.cell(0, 0);
    auto result = mu_reach(g, goal, safe, start);

    EXPECT_FALSE(result.winning_region.empty());
    EXPECT_TRUE(result.winning_region.count(start));
    EXPECT_FALSE(result.path.empty());
    EXPECT_EQ(result.path.front(), start);
    EXPECT_EQ(result.path.back(), g.cell(0, 4));
}

// ---------------------------------------------------------------------------
// Test 2: complete wall — no solution
// ---------------------------------------------------------------------------
TEST(MuCalculus, WallBlocksPath) {
    // Wall at column 2 with no gap: start (col 0) cannot reach goal (col 4)
    auto g = make_walled_grid(5, 5, 2);

    std::unordered_set<CellIdx> goal = { g.cell(2, 4) };
    std::unordered_set<CellIdx> safe;
    for (CellIdx i = 0; i < 25; ++i)
        if (!g.obstacle[i]) safe.insert(i);

    CellIdx start = g.cell(2, 0);
    auto result = mu_reach(g, goal, safe, start);

    EXPECT_TRUE(result.path.empty());
    EXPECT_FALSE(result.winning_region.count(start));
}

// ---------------------------------------------------------------------------
// Test 3: start is already at goal
// ---------------------------------------------------------------------------
TEST(MuCalculus, StartIsGoal) {
    auto g = make_free_grid(3, 3);

    CellIdx start = g.cell(1, 1);
    std::unordered_set<CellIdx> goal = { start };
    std::unordered_set<CellIdx> safe;
    for (CellIdx i = 0; i < 9; ++i) safe.insert(i);

    auto result = mu_reach(g, goal, safe, start);

    EXPECT_FALSE(result.path.empty());
    EXPECT_EQ(result.path.size(), 1u);
    EXPECT_EQ(result.path.front(), start);
}

// ---------------------------------------------------------------------------
// Test 4: fixed-point reaches all free cells in connected free grid
// ---------------------------------------------------------------------------
TEST(MuCalculus, WinningRegionCoversAllFree) {
    auto g = make_free_grid(4, 4);

    // Goal = bottom-right corner
    std::unordered_set<CellIdx> goal = { g.cell(3, 3) };
    std::unordered_set<CellIdx> safe;
    for (CellIdx i = 0; i < 16; ++i) safe.insert(i);

    auto result = mu_reach(g, goal, safe, g.cell(0, 0));

    // Every free cell should be in the winning region (all can reach the corner)
    EXPECT_EQ(result.winning_region.size(), 16u);
}

// ---------------------------------------------------------------------------
// Test 5: epistemic lift — sensing waypoints appear in result
// ---------------------------------------------------------------------------
TEST(MuCalculus, EpistemicSensingWaypoints) {
    auto g = make_free_grid(6, 1);   // single row: 0-1-2-3-4-5

    std::unordered_set<CellIdx> goal    = { 5u };
    std::unordered_set<CellIdx> safe;
    for (CellIdx i = 0; i < 6; ++i) safe.insert(i);

    // Sensing possible only at cell 4 (adjacent to goal)
    std::unordered_set<CellIdx> sensing = { 4u };

    auto result = mu_reach_epistemic(g, goal, safe, sensing, 0u);

    EXPECT_FALSE(result.path.empty());
    // Cell 4 should be a sensing waypoint
    bool found = false;
    for (CellIdx c : result.sensing_waypoints)
        if (c == 4u) { found = true; break; }
    EXPECT_TRUE(found);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
