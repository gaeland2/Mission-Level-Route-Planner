#include <gtest/gtest.h>

#include <cmath>
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
  Fixture(int nx, int ny, int nz, double res = 1.0, int connectivity = 26)
  : grid(make_shared<OccupancyGrid>(nx, ny, nz, res)),
    ctx(grid, make_shared<Dijkstra>(grid, connectivity)) {}
};

int count_not_free(const OccupancyGrid & g, int nx, int ny, int nz)
{
  int c = 0;
  for (int i = 0; i < nx; ++i) {
    for (int j = 0; j < ny; ++j) {
      for (int k = 0; k < nz; ++k) {
        if (!g.is_free({i, j, k})) {++c;}
      }
    }
  }
  return c;
}
}  // namespace

TEST(SafetyMargin, BlocksASphereAroundAnObstacle)
{
  Fixture f(21, 21, 21);
  f.grid->block({10, 10, 10});
  f.grid->set_safety_margin(2.0);
  EXPECT_FALSE(f.grid->is_free({12, 10, 10}));   // distance 2
  EXPECT_FALSE(f.grid->is_free({10, 10, 8}));    // distance 2
  EXPECT_FALSE(f.grid->is_free({11, 11, 11}));   // distance 1.73
  EXPECT_TRUE(f.grid->is_free({12, 11, 10}));    // distance 2.24
  EXPECT_TRUE(f.grid->is_free({12, 12, 10}));    // distance 2.83
  EXPECT_EQ(count_not_free(*f.grid, 21, 21, 21), 33);  // lattice points with x^2+y^2+z^2 <= 4
}

TEST(SafetyMargin, IsRoundNotCubic)
{
  Fixture f(21, 21, 21);
  f.grid->block({10, 10, 10});
  f.grid->set_safety_margin(1.5);
  EXPECT_FALSE(f.grid->is_free({11, 11, 10}));  // 1.41, inside
  EXPECT_TRUE(f.grid->is_free({11, 11, 11}));   // 1.73, a cube would have blocked this corner
}

TEST(SafetyMargin, UsesMetersNotCells)
{
  Fixture f(21, 21, 21, 0.5);
  f.grid->block({10, 10, 10});
  f.grid->set_safety_margin(1.0);  // 2 cells at 0.5 m
  EXPECT_FALSE(f.grid->is_free({12, 10, 10}));
  EXPECT_TRUE(f.grid->is_free({13, 10, 10}));
}

TEST(SafetyMargin, ZeroMarginBlocksOnlyTheObstacle)
{
  Fixture f(11, 11, 11);
  f.grid->block({5, 5, 5});
  f.grid->set_safety_margin(0.0);
  EXPECT_EQ(count_not_free(*f.grid, 11, 11, 11), 1);
}

TEST(SafetyMargin, MarginSetBeforeObstaclesAlsoWorks)
{
  Fixture f(11, 11, 11);
  f.grid->set_safety_margin(1.0);
  f.grid->block({5, 5, 5});
  EXPECT_FALSE(f.grid->is_free({6, 5, 5}));
  EXPECT_TRUE(f.grid->is_free({7, 5, 5}));
}

TEST(SafetyMargin, IsOccupiedIgnoresTheMargin)
{
  Fixture f(11, 11, 11);
  f.grid->block({5, 5, 5});
  f.grid->set_safety_margin(2.0);
  EXPECT_TRUE(f.grid->is_occupied({5, 5, 5}));
  EXPECT_FALSE(f.grid->is_occupied({6, 5, 5}));
  EXPECT_FALSE(f.grid->is_free({6, 5, 5}));
}

TEST(SafetyMargin, ChangingTheMarginRebuildsIt)
{
  Fixture f(15, 15, 15);
  f.grid->block({7, 7, 7});
  f.grid->set_safety_margin(3.0);
  EXPECT_FALSE(f.grid->is_free({9, 7, 7}));
  f.grid->set_safety_margin(1.0);
  EXPECT_TRUE(f.grid->is_free({9, 7, 7}));
  EXPECT_FALSE(f.grid->is_free({8, 7, 7}));
  f.grid->set_safety_margin(0.0);
  EXPECT_TRUE(f.grid->is_free({8, 7, 7}));
  EXPECT_FALSE(f.grid->is_free({7, 7, 7}));
}

TEST(SafetyMargin, UnblockKeepsCellsCoveredByAnotherObstacle)
{
  Fixture f(15, 15, 1);
  f.grid->set_safety_margin(1.0);
  f.grid->block({5, 5, 0});
  f.grid->block({7, 5, 0});
  EXPECT_FALSE(f.grid->is_free({6, 5, 0}));  // covered by both
  f.grid->unblock({5, 5, 0});
  EXPECT_FALSE(f.grid->is_free({6, 5, 0}));  // still covered by the second one
  EXPECT_TRUE(f.grid->is_free({4, 5, 0}));   // only the first one covered this
  f.grid->unblock({7, 5, 0});
  EXPECT_TRUE(f.grid->is_free({6, 5, 0}));
}

TEST(SafetyMargin, BlockingTheSameCellTwiceDoesNotDoubleCount)
{
  Fixture f(11, 11, 1);
  f.grid->set_safety_margin(1.0);
  f.grid->block({5, 5, 0});
  f.grid->block({5, 5, 0});
  f.grid->unblock({5, 5, 0});
  EXPECT_TRUE(f.grid->is_free({5, 5, 0}));
  EXPECT_TRUE(f.grid->is_free({6, 5, 0}));
}

TEST(SafetyMargin, IsClippedAtTheMapEdge)
{
  Fixture f(10, 10, 10);
  f.grid->block({0, 0, 0});
  f.grid->set_safety_margin(2.0);
  EXPECT_FALSE(f.grid->is_free({1, 1, 0}));
  EXPECT_FALSE(f.grid->is_free({2, 0, 0}));
  EXPECT_TRUE(f.grid->is_free({3, 0, 0}));
}

TEST(SafetyMargin, PathKeepsItsDistanceFromTheObstacle)
{
  Fixture f(30, 30, 1);
  f.grid->block({15, 15, 0});
  f.grid->set_safety_margin(2.5);
  auto r = f.ctx.plan({0, 15, 0}, {29, 15, 0});
  ASSERT_TRUE(r.success());
  for (const auto & n : r.path) {
    const double d = sqrt(pow(n[0] - 15, 2) + pow(n[1] - 15, 2) + pow(n[2], 2));
    EXPECT_GE(d, 2.5);
  }
}

TEST(SafetyMargin, GapNarrowerThanTwiceTheMarginIsClosed)
{
  Fixture f(30, 30, 1);
  for (int y = 0; y < 30; ++y) {
    if (y != 15) {f.grid->block({10, y, 0});}  // wall with a one-cell gap
  }
  EXPECT_TRUE(f.ctx.plan({0, 15, 0}, {29, 15, 0}).success());  // no margin: fits through
  f.grid->set_safety_margin(1.0);
  EXPECT_EQ(f.ctx.plan({0, 15, 0}, {29, 15, 0}).status, PlanStatus::NO_PATH);
  f.grid->set_safety_margin(0.0);
  EXPECT_TRUE(f.ctx.plan({0, 15, 0}, {29, 15, 0}).success());
}

TEST(SafetyMargin, StartInsideTheMarginIsRejected)
{
  Fixture f(20, 20, 1);
  f.grid->block({5, 5, 0});
  f.grid->set_safety_margin(2.0);
  EXPECT_EQ(f.ctx.plan({6, 5, 0}, {15, 15, 0}).status, PlanStatus::START_BLOCKED);
  EXPECT_EQ(f.ctx.plan({15, 15, 0}, {6, 5, 0}).status, PlanStatus::GOAL_BLOCKED);
}

TEST(SafetyMargin, AppliesToObstaclesOutsideTheAltitudeLimits)
{
  Fixture f(10, 10, 10);
  f.grid->block({5, 5, 3});
  f.grid->set_safety_margin(2.0);
  f.grid->set_altitude_limits(5.0, 8.0);  // allowed layers 5..8, the obstacle is below them
  EXPECT_FALSE(f.grid->is_free({5, 5, 5}));  // distance 2 from the obstacle
  EXPECT_TRUE(f.grid->is_free({5, 5, 6}));
}

TEST(SafetyMargin, NegativeMarginThrows)
{
  Fixture f(10, 10, 10);
  EXPECT_THROW(f.grid->set_safety_margin(-0.5), invalid_argument);
}

TEST(SafetyMargin, HugeMarginThrowsInsteadOfOverflowing)
{
  Fixture f(5, 5, 5, 0.1);
  EXPECT_THROW(f.grid->set_safety_margin(50.0), invalid_argument);
}
