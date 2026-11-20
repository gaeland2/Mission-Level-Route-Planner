#include <gtest/gtest.h>

#include <limits>
#include <memory>
#include <stdexcept>

#include "mission_level_route_planner/grid.h"
#include "mission_level_route_planner/mission_planner.h"

using namespace std;

using namespace mission_level_route_planner;

namespace
{
// 10 x 10 x 1 grid, 0.5 m cells, origin at (10, 20, 0) so world != index coordinates
// Cell (i,j,0) has its center at (10.25 + 0.5 i, 20.25 + 0.5 j, 0.25)
shared_ptr<OccupancyGrid> make_grid()
{
  return make_shared<OccupancyGrid>(10, 10, 1, 0.5, Waypoint{10.0, 20.0, 0.0});
}
} 
//kevin's tests
TEST(MissionPlannerValidate, NegativeOriginGridAcceptsFreePoses)
{
  auto g = make_shared<OccupancyGrid>(10, 10, 10, 1.0, Waypoint{-5.0, -5.0, -5.0});
  MissionPlanner p;
  p.set_map(g);
  EXPECT_EQ(p.validate({-4.5, -4.5, -4.5}, {4.5, 4.5, 4.5}), PlanStatus::OK);
}


TEST(MissionPlannerPath, FindsPathOnOpenGrid)
{
  MissionPlanner p;
  p.set_map(make_grid());
  const PlanRequest req{{10.25, 20.25, 0.25}, {14.25, 24.25, 0.25}};
  const auto path = p.getMissionPath(req);
  ASSERT_FALSE(path.empty());
  EXPECT_NEAR(path.front().x, 10.25, 1e-6);
  EXPECT_NEAR(path.front().y, 20.25, 1e-6);
  EXPECT_NEAR(path.back().x, 14.25, 1e-6);
  EXPECT_NEAR(path.back().y, 24.25, 1e-6);
}

TEST(MissionPlannerPath, InvalidRequestGivesEmptyPath)
{
  MissionPlanner p;
  p.set_map(make_grid());
  const PlanRequest req{{10.25, 20.25, 0.25}, {99, 99, 0.25}};
  EXPECT_TRUE(p.getMissionPath(req).empty());
}

TEST(MissionPlannerPath, NoMapThrows)
{
  MissionPlanner p;
  const PlanRequest req{{0, 0, 0}, {1, 1, 0}};
  EXPECT_THROW(p.getMissionPath(req), invalid_argument);
}


TEST(MissionPlanner, ReportsMissingMap)
{
  MissionPlanner planner;

  EXPECT_EQ(
    planner.validate(
      Waypoint{0.0, 0.0, 0.0},
      Waypoint{1.0, 1.0, 1.0}),
    PlanStatus::NO_MAP);
}

TEST(MissionPlanner, AcceptsFreeWorldPoses)
{
  auto grid = std::make_shared<OccupancyGrid>(
    10, 10, 10, 1.0, Waypoint{-5.0, -5.0, -5.0});

  MissionPlanner planner;
  planner.set_map(grid);

  EXPECT_TRUE(planner.has_map());
  EXPECT_EQ(
    planner.validate(
      Waypoint{-4.5, -4.5, -4.5},
      Waypoint{4.5, 4.5, 4.5}),
    PlanStatus::OK);
}

TEST(MissionPlanner, RejectsNonFinitePose)
{
  auto grid = std::make_shared<OccupancyGrid>(10, 10, 10);
  MissionPlanner planner;
  planner.set_map(grid);

  const double nan = std::numeric_limits<double>::quiet_NaN();

  EXPECT_EQ(
    planner.validate(
      Waypoint{nan, 0.0, 0.0},
      Waypoint{1.0, 1.0, 1.0}),
    PlanStatus::INVALID_POSE);
}


TEST(MissionPlanner, RejectsOutOfBoundsPose)
{
  auto grid = std::make_shared<OccupancyGrid>(10, 10, 10);
  MissionPlanner planner;
  planner.set_map(grid);

  EXPECT_EQ(
    planner.validate(
      Waypoint{-0.1, 0.0, 0.0},
      Waypoint{1.0, 1.0, 1.0}),
    PlanStatus::OUT_OF_BOUNDS);
}

TEST(MissionPlanner, RejectsBlockedStart)
{
  auto grid = std::make_shared<OccupancyGrid>(10, 10, 10);
  grid->block({2, 2, 2});

  MissionPlanner planner;
  planner.set_map(grid);

  EXPECT_EQ(
    planner.validate(
      grid->to_world({2, 2, 2}),
      grid->to_world({8, 8, 8})),
    PlanStatus::START_BLOCKED);
}

TEST(MissionPlanner, RejectsBlockedGoal)
{
  auto grid = std::make_shared<OccupancyGrid>(10, 10, 10);
  grid->block({8, 8, 8});

  MissionPlanner planner;
  planner.set_map(grid);

  EXPECT_EQ(
    planner.validate(
      grid->to_world({2, 2, 2}),
      grid->to_world({8, 8, 8})),
    PlanStatus::GOAL_BLOCKED);
}

TEST(MissionPlanner, RejectsNullMap)
{
  MissionPlanner planner;

  EXPECT_THROW(
    planner.set_map(nullptr),
    invalid_argument);
}


// Validating before any map is set reports NO_MAP
TEST(MissionPlannerValidate, NoMapReportsNoMap)
{
  MissionPlanner p;
  EXPECT_FALSE(p.has_map());
  EXPECT_EQ(p.validate({0, 0, 0}, {1, 1, 0}), PlanStatus::NO_MAP);
}

// set_map(nullptr) throws and leaves the planner without a map
TEST(MissionPlannerValidate, NullMapIsRejected)
{
  MissionPlanner p;
  EXPECT_THROW(p.set_map(nullptr), invalid_argument);
  EXPECT_FALSE(p.has_map());
}

// Free start and goal given in meters (on an offset-origin grid) validate as OK
TEST(MissionPlannerValidate, FreeStartAndGoalInMetersAreOk)
{
  MissionPlanner p;
  p.set_map(make_grid());
  EXPECT_TRUE(p.has_map());
  EXPECT_EQ(p.validate({10.25, 20.25, 0.25}, {14.25, 24.25, 0.25}), PlanStatus::OK);
}

// A pose anywhere inside a cell is treated as that cell (checks the cell boundary)
TEST(MissionPlannerValidate, PoseInsideACellCountsAsThatCell)
{
  auto g = make_grid();
  g->block({2, 2, 0});  // covers x 11.0..11.5, y 21.0..21.5
  MissionPlanner p;
  p.set_map(g);
  EXPECT_EQ(p.validate({11.01, 21.49, 0.1}, {10.25, 20.25, 0.25}), PlanStatus::START_BLOCKED);
  EXPECT_EQ(p.validate({10.99, 21.49, 0.1}, {10.25, 20.25, 0.25}), PlanStatus::OK);  // next cell
}

// Start and goal in the same free cell is valid
TEST(MissionPlannerValidate, StartEqualsGoalIsOk)
{
  MissionPlanner p;
  p.set_map(make_grid());
  EXPECT_EQ(p.validate({12.25, 22.25, 0.25}, {12.25, 22.25, 0.25}), PlanStatus::OK);
}

// Poses outside the map on every side (x, y, above, below) give OUT_OF_BOUNDS,
// including just-past-the-edge cases
TEST(MissionPlannerValidate, PoseOutsideMapIsOutOfBounds)
{
  MissionPlanner p;
  p.set_map(make_grid());
  const Waypoint ok{10.25, 20.25, 0.25};
  EXPECT_EQ(p.validate({0, 0, 0}, ok), PlanStatus::OUT_OF_BOUNDS);            // start
  EXPECT_EQ(p.validate(ok, {99, 99, 0.25}), PlanStatus::OUT_OF_BOUNDS);       // goal
  EXPECT_EQ(p.validate(ok, {15.0, 20.25, 0.25}), PlanStatus::OUT_OF_BOUNDS);  // just past x edge
  EXPECT_EQ(p.validate(ok, {9.99, 20.25, 0.25}), PlanStatus::OUT_OF_BOUNDS);  // just before origin
  EXPECT_EQ(p.validate(ok, {10.25, 20.25, 5.0}), PlanStatus::OUT_OF_BOUNDS);  // above the map
  EXPECT_EQ(p.validate(ok, {10.25, 20.25, -1.0}), PlanStatus::OUT_OF_BOUNDS); // below the map
}


// NaN and infinity give INVALID_POSE; a huge but finite value is simply out of bounds
TEST(MissionPlannerValidate, NonFinitePoseIsRejectedNotCrashed)
{
  MissionPlanner p;
  p.set_map(make_grid());
  const double nan = numeric_limits<double>::quiet_NaN();
  const double inf = numeric_limits<double>::infinity();
  const Waypoint ok{10.25, 20.25, 0.25};
  EXPECT_EQ(p.validate({nan, 20.25, 0.25}, ok), PlanStatus::INVALID_POSE);
  EXPECT_EQ(p.validate(ok, {10.25, -inf, 0.25}), PlanStatus::INVALID_POSE);
  EXPECT_EQ(p.validate({1e300, 20.25, 0.25}, ok), PlanStatus::OUT_OF_BOUNDS);
}

// A start inside an obstacle gives START_BLOCKED; a goal inside one gives GOAL_BLOCKED.
TEST(MissionPlannerValidate, BlockedStartAndGoalAreReported)
{
  auto g = make_grid();
  g->block({2, 2, 0});  // center (11.25, 21.25)
  g->block({7, 7, 0});  // center (13.75, 23.75)
  MissionPlanner p;
  p.set_map(g);
  const Waypoint free_pt{10.25, 20.25, 0.25};
  EXPECT_EQ(p.validate({11.25, 21.25, 0.25}, free_pt), PlanStatus::START_BLOCKED);
  EXPECT_EQ(p.validate(free_pt, {13.75, 23.75, 0.25}), PlanStatus::GOAL_BLOCKED);
}

// If both are blocked, START_BLOCKED is reported
TEST(MissionPlannerValidate, BothBlockedReportsStartFirst)
{
  auto g = make_grid();
  g->block({2, 2, 0});
  g->block({7, 7, 0});
  MissionPlanner p;
  p.set_map(g);
  EXPECT_EQ(p.validate({11.25, 21.25, 0.25}, {13.75, 23.75, 0.25}), PlanStatus::START_BLOCKED);
}

// Out-of-bounds is reported before blocked when both problems exist
TEST(MissionPlannerValidate, OutOfBoundsTakesPriorityOverBlocked)
{
  auto g = make_grid();
  g->block({2, 2, 0});
  MissionPlanner p;
  p.set_map(g);
  EXPECT_EQ(p.validate({11.25, 21.25, 0.25}, {99, 99, 0.25}), PlanStatus::OUT_OF_BOUNDS);
}

// A pose inside an obstacle's safety margin counts as blocked; one outside does not
TEST(MissionPlannerValidate, SafetyMarginMakesNearbyPoseBlocked)
{
  auto g = make_grid();
  g->block({5, 5, 0});             // center (12.75, 22.75)
  g->set_safety_margin(0.6);       // reaches the next cell (0.5 m away)
  MissionPlanner p;
  p.set_map(g);
  const Waypoint free_pt{10.25, 20.25, 0.25};
  EXPECT_EQ(p.validate({13.25, 22.75, 0.25}, free_pt), PlanStatus::START_BLOCKED);
  EXPECT_EQ(p.validate({14.25, 22.75, 0.25}, free_pt), PlanStatus::OK);
}

//Poses outside the grid's altitude limits count as blocked
TEST(MissionPlannerValidate, AltitudeLimitsApply)
{
  auto g = make_shared<OccupancyGrid>(4, 4, 4, 1.0);  // z layers centered at 0.5, 1.5, 2.5, 3.5
  g->set_altitude_limits(1.0, 3.0);                   // centers 1.5 and 2.5 qualify: layers 1, 2
  MissionPlanner p;
  p.set_map(g);
  EXPECT_EQ(p.validate({0.5, 0.5, 1.5}, {1.5, 1.5, 2.5}), PlanStatus::OK);
  EXPECT_EQ(p.validate({0.5, 0.5, 0.5}, {1.5, 1.5, 2.5}), PlanStatus::START_BLOCKED);
  EXPECT_EQ(p.validate({0.5, 0.5, 1.5}, {1.5, 1.5, 3.5}), PlanStatus::GOAL_BLOCKED);
}

TEST(MissionPlannerValidate, NewMapReplacesOldOne)
{
  auto blocked = make_grid();
  blocked->block({0, 0, 0});
  MissionPlanner p;
  p.set_map(blocked);
  const Waypoint s{10.25, 20.25, 0.25}, goal{14.25, 24.25, 0.25};
  EXPECT_EQ(p.validate(s, goal), PlanStatus::START_BLOCKED);

  p.set_map(make_grid());
  EXPECT_EQ(p.validate(s, goal), PlanStatus::OK);
}

TEST(MissionPlannerValidate, RequestOverloadMatches)
{
  MissionPlanner p;
  p.set_map(make_grid());
  const PlanRequest good{{10.25, 20.25, 0.25}, {12.25, 22.25, 0.25}};
  const PlanRequest bad{{10.25, 20.25, 0.25}, {99, 99, 0.25}};
  EXPECT_EQ(p.validate(good), PlanStatus::OK);
  EXPECT_EQ(p.validate(bad), PlanStatus::OUT_OF_BOUNDS);
}

TEST(PlanStatusText, ToString)
{
  EXPECT_STREQ(to_string(PlanStatus::OK), "OK");
  EXPECT_STREQ(to_string(PlanStatus::NO_MAP), "NO_MAP");
  EXPECT_STREQ(to_string(PlanStatus::OUT_OF_BOUNDS), "OUT_OF_BOUNDS");
  EXPECT_STREQ(to_string(PlanStatus::START_BLOCKED), "START_BLOCKED");
  EXPECT_STREQ(to_string(PlanStatus::GOAL_BLOCKED), "GOAL_BLOCKED");
  EXPECT_STREQ(to_string(PlanStatus::INVALID_POSE), "INVALID_POSE");
}
