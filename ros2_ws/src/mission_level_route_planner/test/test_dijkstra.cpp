#include <gtest/gtest.h>

#include <cmath>
#include <memory>

#include "mission_level_route_planner/context.h"
#include "mission_level_route_planner/dijkstra.h"
#include "mission_level_route_planner/grid.h"

using namespace std;

using namespace mission_level_route_planner;  // NOLINT

namespace
{
struct Fixture
{
  shared_ptr<OccupancyGrid> grid;
  Context ctx;
  Fixture(int nx, int ny, int nz, int connectivity = 6, double res = 1.0)
  : grid(make_shared<OccupancyGrid>(nx, ny, nz, res)),
    ctx(grid, make_shared<Dijkstra>(grid, connectivity)) {}
};

// A wall at x = 5 across the whole map, optionally with a gap at y = gap_y.
void build_wall(OccupancyGrid & g, int ny, int gap_y = -1)
{
  for (int y = 0; y < ny; ++y) {
    if (y != gap_y) {g.block({5, y, 0});}
  }
}
}  // namespace

TEST(Dijkstra, StraightLineOnEmptyGrid)
{
  Fixture f(10, 10, 1);
  auto r = f.ctx.plan({0, 0, 0}, {5, 0, 0});
  ASSERT_TRUE(r.success());
  EXPECT_EQ(r.path.front(), (Index{0, 0, 0}));
  EXPECT_EQ(r.path.back(), (Index{5, 0, 0}));
  EXPECT_DOUBLE_EQ(r.length_m, 5.0);
}

TEST(Dijkstra, StartEqualsGoal)
{
  Fixture f(10, 10, 1);
  auto r = f.ctx.plan({3, 3, 0}, {3, 3, 0});
  ASSERT_TRUE(r.success());
  EXPECT_EQ(r.path.size(), 1u);
  EXPECT_DOUBLE_EQ(r.length_m, 0.0);
}

TEST(Dijkstra, DetoursThroughGapInWall)
{
  Fixture f(10, 10, 1);
  build_wall(*f.grid, 10, 8);
  auto r = f.ctx.plan({0, 0, 0}, {9, 0, 0});
  ASSERT_TRUE(r.success());
  EXPECT_DOUBLE_EQ(r.length_m, 9 + 8 + 8);  // up to the gap, across, and back down
}

TEST(Dijkstra, PathNeverTouchesBlockedCells)
{
  Fixture f(10, 10, 1, 26);
  build_wall(*f.grid, 10, 8);
  auto r = f.ctx.plan({0, 0, 0}, {9, 0, 0});
  ASSERT_TRUE(r.success());
  for (const auto & n : r.path) {EXPECT_TRUE(f.grid->is_free(n));}
}

TEST(Dijkstra, UnreachableGoalReportsNoPath)
{
  Fixture f(10, 10, 1);
  build_wall(*f.grid, 10);  // no gap
  auto r = f.ctx.plan({0, 0, 0}, {9, 0, 0});
  EXPECT_EQ(r.status, PlanStatus::NO_PATH);
  EXPECT_TRUE(r.path.empty());
}

TEST(Dijkstra, StartBlocked)
{
  Fixture f(10, 10, 1);
  f.grid->block({0, 0, 0});
  EXPECT_EQ(f.ctx.plan({0, 0, 0}, {5, 5, 0}).status, PlanStatus::START_BLOCKED);
}

TEST(Dijkstra, GoalBlocked)
{
  Fixture f(10, 10, 1);
  f.grid->block({5, 5, 0});
  EXPECT_EQ(f.ctx.plan({0, 0, 0}, {5, 5, 0}).status, PlanStatus::GOAL_BLOCKED);
}

TEST(Dijkstra, OutOfBounds)
{
  Fixture f(10, 10, 1);
  EXPECT_EQ(f.ctx.plan({0, 0, 0}, {50, 0, 0}).status, PlanStatus::OUT_OF_BOUNDS);
  EXPECT_EQ(f.ctx.plan({-1, 0, 0}, {5, 0, 0}).status, PlanStatus::OUT_OF_BOUNDS);
}

TEST(Dijkstra, FliesOverObstacleIn3D)
{
  Fixture f(10, 10, 3);
  build_wall(*f.grid, 10);  // blocks the ground layer only
  auto r = f.ctx.plan({0, 0, 0}, {9, 0, 0});
  ASSERT_TRUE(r.success());
  bool went_up = false;
  for (const auto & n : r.path) {went_up = went_up || n[2] > 0;}
  EXPECT_TRUE(went_up);
}

TEST(Dijkstra, DiagonalsAreShorterWith26Connectivity)
{
  Fixture f6(10, 10, 1, 6);
  Fixture f26(10, 10, 1, 26);
  EXPECT_DOUBLE_EQ(f6.ctx.plan({0, 0, 0}, {4, 4, 0}).length_m, 8.0);
  EXPECT_NEAR(f26.ctx.plan({0, 0, 0}, {4, 4, 0}).length_m, 4 * sqrt(2.0), 1e-9);
}

TEST(Dijkstra, ResolutionScalesLength)
{
  Fixture f(10, 10, 1, 6, 0.5);
  EXPECT_DOUBLE_EQ(f.ctx.plan({0, 0, 0}, {4, 0, 0}).length_m, 2.0);
}

TEST(Dijkstra, MetricsAreFilledIn)
{
  Fixture f(10, 10, 1);
  auto r = f.ctx.plan({0, 0, 0}, {5, 5, 0});
  EXPECT_GE(r.compute_time_s, 0.0);
  EXPECT_GT(r.nodes_expanded, 0);
}

TEST(Dijkstra, ReplanAfterMapChange)
{
  Fixture f(10, 10, 1);
  EXPECT_TRUE(f.ctx.plan({0, 0, 0}, {9, 0, 0}).success());
  build_wall(*f.grid, 10);  // a new obstacle appears
  EXPECT_EQ(f.ctx.plan({0, 0, 0}, {9, 0, 0}).status, PlanStatus::NO_PATH);
  f.grid->unblock({5, 4, 0});
  EXPECT_TRUE(f.ctx.plan({0, 0, 0}, {9, 0, 0}).success());
}

TEST(Grid, WorldIndexRoundTrip)
{
  OccupancyGrid g(10, 10, 5, 0.5, Waypoint{-2.0, 3.0, 0.0});
  const Index n{4, 7, 2};
  EXPECT_EQ(g.to_index(g.to_world(n)), n);
  EXPECT_EQ(g.from_id(g.to_id(n)), n);
}
