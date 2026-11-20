
#include <gtest/gtest.h>

#include <memory>

#include "mission_level_route_planner/dijkstra.h"
#include "mission_level_route_planner/grid.h"

using namespace std;
using namespace mission_level_route_planner;

TEST(DijkstraGuard, MaxIterationsReached)
{
  auto grid = make_shared<OccupancyGrid>(20, 20, 1);

  // Allow only one node expansion.
  Dijkstra planner(grid, 6, 1, 5.0);

  auto result = planner.execute_pathfinding({0, 0, 0}, {19, 19, 0});

  EXPECT_EQ(result.status, PlanStatus::MAX_ITERATIONS_REACHED);
  EXPECT_EQ(result.nodes_expanded, 1);
  EXPECT_TRUE(result.path.empty());
}

TEST(DijkstraGuard, TimeoutReached)
{
  auto grid = make_shared<OccupancyGrid>(20, 20, 1);

  // Set the timeout to zero seconds.
  Dijkstra planner(grid, 6, 100000, 0.0);

  auto result = planner.execute_pathfinding({0, 0, 0}, {19, 19, 0});

  EXPECT_EQ(result.status, PlanStatus::TIMEOUT);
  EXPECT_EQ(result.nodes_expanded, 0);
  EXPECT_TRUE(result.path.empty());
}

TEST(DijkstraGuard, GoalReachedAtIterationLimit)
{
  auto grid = make_shared<OccupancyGrid>(2, 1, 1);

  // The goal requires two node expansions.
  Dijkstra planner(grid, 6, 2, 5.0);

  auto result = planner.execute_pathfinding({0, 0, 0}, {1, 0, 0});

  EXPECT_EQ(result.status, PlanStatus::OK);
  EXPECT_EQ(result.nodes_expanded, 2);
}
