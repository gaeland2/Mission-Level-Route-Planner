#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <vector>
#include <stdexcept>

#include "mission_level_route_planner/grid.h"
#include "mission_level_route_planner/path_smoother.h"

using namespace std;
using namespace mission_level_route_planner;

TEST(PathSmoother, RemovesUnnecessaryWaypoints)
{
  auto grid = make_shared<OccupancyGrid>(10, 10, 1);
  PathSmoother smoother(grid);

  vector<Index> original = {
    {0, 0, 0},
    {1, 0, 0},
    {2, 0, 0},
    {3, 0, 0},
    {4, 0, 0}
  };

  auto simplified = smoother.simplify(original);

  EXPECT_EQ(simplified.size(), 2u);
  EXPECT_EQ(simplified.front(), original.front());
  EXPECT_EQ(simplified.back(), original.back());
}

TEST(PathSmoother, AvoidsBlockedCells)
{
  auto grid = make_shared<OccupancyGrid>(5, 5, 1);

  // Block the direct route between start and goal.
  grid->block({2, 2, 0});

  PathSmoother smoother(grid);

  vector<Index> original = {
    {0, 2, 0},
    {0, 1, 0},
    {0, 0, 0},
    {1, 0, 0},
    {2, 0, 0},
    {3, 0, 0},
    {4, 0, 0},
    {4, 1, 0},
    {4, 2, 0}
  };

  auto simplified = smoother.simplify(original);

  EXPECT_EQ(simplified.front(), original.front());
  EXPECT_EQ(simplified.back(), original.back());

  // The blocked cell must not appear as a waypoint.
  for (const auto & point : simplified) {
    EXPECT_TRUE(grid->is_free(point));
  }

  // The direct shortcut through the obstacle must be rejected.
  EXPECT_GT(simplified.size(), 2u);
}

TEST(PathSmoother, RespectsLengthTolerance)
{
  auto grid = make_shared<OccupancyGrid>(10, 10, 1);
  PathSmoother smoother(grid);

  vector<Index> original = {
    {0, 0, 0},
    {1, 0, 0},
    {2, 0, 0},
    {2, 1, 0},
    {2, 2, 0}
  };

  const double tolerance = 0.05;
  auto simplified = smoother.simplify(original, tolerance);

  // Original path length is 4 meters.
  const double original_length = 4.0;

  double simplified_length = 0.0;

  for (size_t i = 1; i < simplified.size(); ++i) {
    const double dx = simplified[i][0] - simplified[i - 1][0];
    const double dy = simplified[i][1] - simplified[i - 1][1];
    const double dz = simplified[i][2] - simplified[i - 1][2];

    simplified_length +=
      std::sqrt(dx * dx + dy * dy + dz * dz) * grid->resolution();
  }

  EXPECT_LE(simplified_length, original_length * (1.0 + tolerance));
}

TEST(PathSmoother, RejectsDiagonalCornerCollision)
{
  auto grid = make_shared<OccupancyGrid>(3, 3, 1);

  // This obstacle touches the diagonal shortcut at a corner.
  grid->block({1, 0, 0});

  PathSmoother smoother(grid);

  vector<Index> original = {
    {0, 0, 0},
    {0, 1, 0},
    {0, 2, 0},
    {1, 2, 0},
    {2, 2, 0}
  };

  auto simplified = smoother.simplify(original);

  EXPECT_EQ(simplified.front(), original.front());
  EXPECT_EQ(simplified.back(), original.back());

  // The direct diagonal would touch the blocked cell.
  // Therefore, it must not be accepted.
  EXPECT_GT(simplified.size(), 2u);
}

TEST(PathSmoother, Avoids3DObstacles)
{
  auto grid = make_shared<OccupancyGrid>(5, 5, 5);

  // Place an obstacle along the direct 3D diagonal.
  grid->block({1, 1, 1});

  PathSmoother smoother(grid);

  vector<Index> original = {
    {0, 0, 0},
    {0, 0, 1},
    {0, 0, 2},
    {0, 1, 2},
    {0, 2, 2},
    {1, 2, 2},
    {2, 2, 2}
  };

  auto simplified = smoother.simplify(original);

  EXPECT_EQ(simplified.front(), original.front());
  EXPECT_EQ(simplified.back(), original.back());

  // The direct shortcut passes through the blocked voxel.
  EXPECT_GT(simplified.size(), 2u);
}

TEST(PathSmoother, RejectsBlockedTwoPointPath)
{
  auto grid = make_shared<OccupancyGrid>(5, 5, 1);

  // Obstacle directly between the start and destination.
  grid->block({2, 2, 0});

  PathSmoother smoother(grid);

  vector<Index> original = {
    {0, 2, 0},
    {4, 2, 0}
  };

  // A two-point path crossing an obstacle is not safe.
  EXPECT_THROW(
    smoother.simplify(original),
    std::invalid_argument
  );
}