#include <gtest/gtest.h>

#include <limits>
#include <memory>
#include <stdexcept>

#include "mission_level_route_planner/grid.h"
#include "mission_level_route_planner/mission_planner.h"

using namespace mission_level_route_planner;

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
    std::invalid_argument);
}
