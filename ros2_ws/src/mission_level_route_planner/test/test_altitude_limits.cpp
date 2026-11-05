#include <gtest/gtest.h>

#include <memory>
#include <stdexcept>

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
  Fixture(int nx, int ny, int nz, double res = 1.0, Waypoint origin = Waypoint{})
  : grid(make_shared<OccupancyGrid>(nx, ny, nz, res, origin)),
    ctx(grid, make_shared<Dijkstra>(grid, 26)) {}
};
}  // namespace

TEST(AltitudeLimits, OnlyLayersInsideTheRangeAreFree)
{
  Fixture f(10, 10, 10);
  f.grid->set_altitude_limits(2.0, 5.0);  // layer centers 2.5, 3.5, 4.5 are inside
  EXPECT_FALSE(f.grid->is_free({0, 0, 1}));
  EXPECT_TRUE(f.grid->is_free({0, 0, 2}));
  EXPECT_TRUE(f.grid->is_free({0, 0, 4}));
  EXPECT_FALSE(f.grid->is_free({0, 0, 5}));  // center 5.5 is above the ceiling
}

TEST(AltitudeLimits, LimitsRespectResolutionAndOrigin)
{
  Fixture f(10, 10, 10, 0.5, Waypoint{0.0, 0.0, 1.0});  // layer k center = 1.0 + (k + 0.5) * 0.5
  f.grid->set_altitude_limits(2.0, 3.0);
  EXPECT_FALSE(f.grid->is_free({0, 0, 1}));  // 1.75
  EXPECT_TRUE(f.grid->is_free({0, 0, 2}));   // 2.25
  EXPECT_TRUE(f.grid->is_free({0, 0, 3}));   // 2.75
  EXPECT_FALSE(f.grid->is_free({0, 0, 4}));  // 3.25
}

TEST(AltitudeLimits, PathStaysInsideTheLimits)
{
  Fixture f(10, 10, 10);
  f.grid->set_altitude_limits(2.0, 5.0);
  auto r = f.ctx.plan({0, 0, 3}, {9, 9, 3});
  ASSERT_TRUE(r.success());
  for (const auto & n : r.path) {
    EXPECT_GE(n[2], 2);
    EXPECT_LE(n[2], 4);
  }
}

TEST(AltitudeLimits, WallOnlyPassableAboveTheCeilingGivesNoPath)
{
  Fixture f(10, 10, 12);
  for (int y = 0; y < 10; ++y) {
    for (int z = 0; z <= 5; ++z) {f.grid->block({5, y, z});}  // a wall 6 layers tall
  }
  f.grid->set_altitude_limits(0.0, 4.0);  // ceiling below the top of the wall
  EXPECT_EQ(f.ctx.plan({0, 0, 1}, {9, 0, 1}).status, PlanStatus::NO_PATH);

  f.grid->set_altitude_limits(0.0, 9.0);  // raise the ceiling: now it can go over
  auto r = f.ctx.plan({0, 0, 1}, {9, 0, 1});
  ASSERT_TRUE(r.success());
  bool above_wall = false;
  for (const auto & n : r.path) {above_wall = above_wall || n[2] >= 6;}
  EXPECT_TRUE(above_wall);
}

TEST(AltitudeLimits, StartOrGoalOutsideTheLimitsIsRejected)
{
  Fixture f(10, 10, 10);
  f.grid->set_altitude_limits(2.0, 5.0);
  EXPECT_EQ(f.ctx.plan({0, 0, 0}, {5, 5, 3}).status, PlanStatus::START_BLOCKED);
  EXPECT_EQ(f.ctx.plan({0, 0, 3}, {5, 5, 9}).status, PlanStatus::GOAL_BLOCKED);
}

TEST(AltitudeLimits, ClearingTheLimitsRestoresAllLayers)
{
  Fixture f(10, 10, 10);
  f.grid->set_altitude_limits(2.0, 5.0);
  f.grid->clear_altitude_limits();
  EXPECT_TRUE(f.grid->is_free({0, 0, 0}));
  EXPECT_TRUE(f.grid->is_free({0, 0, 9}));
}

TEST(AltitudeLimits, RangeOutsideTheMapLeavesNoFreeCells)
{
  Fixture f(10, 10, 10);
  f.grid->set_altitude_limits(50.0, 60.0);
  EXPECT_FALSE(f.grid->is_free({0, 0, 0}));
  EXPECT_FALSE(f.grid->is_free({0, 0, 9}));
  f.grid->set_altitude_limits(-20.0, -10.0);
  EXPECT_FALSE(f.grid->is_free({0, 0, 0}));
}

TEST(AltitudeLimits, MinAboveMaxThrows)
{
  Fixture f(10, 10, 10);
  EXPECT_THROW(f.grid->set_altitude_limits(5.0, 2.0), invalid_argument);
}

TEST(AltitudeLimits, BlockedCellsStayBlockedInsideTheLimits)
{
  Fixture f(10, 10, 10);
  f.grid->set_altitude_limits(2.0, 5.0);
  f.grid->block({4, 4, 3});
  EXPECT_FALSE(f.grid->is_free({4, 4, 3}));
}
