#include <gtest/gtest.h>

#include <memory>
#include <vector>

#include "mission_level_route_planner/context.h"
#include "mission_level_route_planner/dijkstra.h"
#include "mission_level_route_planner/grid.h"

using namespace std;
using namespace mission_level_route_planner;

class UnsafePathStrategy : public PathfindingStrategy
{
public:
  PlanResult execute_pathfinding(
    const Index & start,
    const Index & goal) override
  {
    PlanResult result;

    result.status = PlanStatus::OK;
    result.path = {start, goal};
    result.length_m = 4.0;

    return result;
  }
};

TEST(PathSmoothingIntegration, OptionalSmoothing)
{
  auto grid = make_shared<OccupancyGrid>(10, 10, 1);

  auto dijkstra = make_shared<Dijkstra>(grid);
  Context context(grid, dijkstra);

  Index start = {0, 0, 0};
  Index goal = {5, 0, 0};

  // Run Dijkstra without smoothing.
  PlanResult original = context.plan(start, goal, false);

  // Run Dijkstra with smoothing enabled.
  PlanResult smoothed = context.plan(start, goal, true);

  ASSERT_TRUE(original.success());
  ASSERT_TRUE(smoothed.success());

  // Smoothing should preserve the start and goal.
  EXPECT_EQ(smoothed.path.front(), start);
  EXPECT_EQ(smoothed.path.back(), goal);

  // Smoothing should not increase the waypoint count.
  EXPECT_LE(smoothed.path.size(), original.path.size());

  // The returned length must remain within 5% of the original.
  EXPECT_LE(smoothed.length_m, original.length_m * 1.05 + 1e-9);

  // On this unobstructed straight route, only two waypoints are needed.
  EXPECT_EQ(smoothed.path.size(), 2u);
}

TEST(PathSmoothingIntegration, RejectsUnsafePath)
{
  auto grid = make_shared<OccupancyGrid>(5, 5, 1);

  // Place an obstacle directly between start and goal.
  grid->block({2, 2, 0});

  auto strategy = make_shared<UnsafePathStrategy>();
  Context context(grid, strategy);

  Index start = {0, 2, 0};
  Index goal = {4, 2, 0};

  // Request smoothing on a deliberately unsafe path.
  PlanResult result = context.plan(start, goal, true);

  // The planner must not report an unsafe path as successful.
  EXPECT_EQ(result.status, PlanStatus::NO_PATH);
  EXPECT_FALSE(result.success());

  // No unsafe waypoints should be returned.
  EXPECT_TRUE(result.path.empty());

  // The reported path length should be reset.
  EXPECT_DOUBLE_EQ(result.length_m, 0.0);
}