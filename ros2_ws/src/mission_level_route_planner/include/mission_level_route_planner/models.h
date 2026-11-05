// Plain data types shared by the whole planner. No ROS includes here.
#pragma once

#include <array>
#include <vector>

using namespace std;

namespace mission_level_route_planner
{

// A grid cell index {i, j, k}. Planners work in index space; the grid converts to meters.
using Index = array<int, 3>;

enum class PlanStatus
{
  OK,
  START_BLOCKED,
  GOAL_BLOCKED,
  OUT_OF_BOUNDS,
  NO_PATH
};

// A position in the map frame, in meters (ENU: x east, y north, z up).
struct Waypoint
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct PlanResult
{
  PlanStatus status{PlanStatus::NO_PATH};
  vector<Index> path;
  double length_m{0.0};
  double compute_time_s{0.0};
  int nodes_expanded{0};

  bool success() const {return status == PlanStatus::OK;}
};

}  // namespace mission_level_route_planner
