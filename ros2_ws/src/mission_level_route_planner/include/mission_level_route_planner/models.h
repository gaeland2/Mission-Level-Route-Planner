// Plain data types that will be used by the whole planner. i dont think ROS is needed here. Add whatever else data types you guys need here

#pragma once

#include <array>
#include <vector>

using namespace std;

namespace mission_level_route_planner {

// grid cell of index {i, j, k}, the planner will work in index space and the grid converts it to meters
using Index = array<int, 3>;

enum class PlanStatus {
  OK,
  START_BLOCKED,
  GOAL_BLOCKED,
  OUT_OF_BOUNDS,
  NO_PATH
};

enum class ServiceStatus {
  RETURN_SUCCESS,
  RETURN_FAILURE
};


// A position in the map frame, in meters (East, north, up: x east, y north, z up).
struct Waypoint {
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct PlanResult {
  PlanStatus status{PlanStatus::NO_PATH};
  vector<Index> path;
  double length_m{0.0};
  double compute_time_s{0.0};
  int nodes_expanded{0};

  bool success() const {return status == PlanStatus::OK;}
};



} // end namespace
