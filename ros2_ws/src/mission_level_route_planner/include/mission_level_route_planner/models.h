// Plain data types shared by the whole planner. No ROS includes here.
#pragma once

#include <array>
#include <vector>
#include <stdexcept>

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
  NO_MAP
};

// turn PlanStatus into text for log messages
inline const char * to_string(PlanStatus s)
{
  switch (s) {
    case PlanStatus::OK: return "OK";
    case PlanStatus::START_BLOCKED: return "START_BLOCKED";
    case PlanStatus::GOAL_BLOCKED: return "GOAL_BLOCKED";
    case PlanStatus::OUT_OF_BOUNDS: return "OUT_OF_BOUNDS";
    case PlanStatus::NO_PATH: return "NO_PATH";
    case PlanStatus::NO_MAP: return "NO_MAP";
  }
  return "UNKNOWN";
}

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
