#pragma once

#include <limits>
#include <memory>

#include "mission_level_route_planner/grid.h"
#include "mission_level_route_planner/models.h"

using namespace std;

namespace mission_level_route_planner
{


class MissionPlanner
{
public:
  MissionPlanner() = default;   // starts with no map (test_mission_planner.cpp was throwing crazy errors)

  explicit MissionPlanner(shared_ptr<OccupancyGrid> grid);

  // Accept a map. Throws invalid_argument if grid is null
  void set_map(shared_ptr<OccupancyGrid> grid);
  
  bool has_map() const {return grid_ != nullptr;}
  shared_ptr<OccupancyGrid> map() const {return grid_;}

  // Check both start & goal poses against the map and return a PlanStatus
  PlanStatus validate(const Waypoint & start, const Waypoint & goal) const;
  // Same check, just taking a PlanRequest instead
  PlanStatus validate(const PlanRequest & request) const
  {
    return validate(request.startPose, request.endPose);
  }

  
// Check both start & goal poses against the map and return a corresponding PlanStatus
  vector<Waypoint> getMissionPath(const PlanRequest & request) const;

  // Same search, but returns the full PlanResult so the caller can see WHY it failed
  // (TIMEOUT, MAX_ITERATIONS_REACHED, NO_PATH, ...). Path is in grid indices.
  PlanResult plan(const PlanRequest & request) const;

  // Search guards passed to Dijkstra. Defaults are generous for a 100 m map.
  void set_search_limits(int max_iterations, double timeout_seconds)
  {
    max_iterations_ = max_iterations;
    timeout_seconds_ = timeout_seconds;
  }

private:
  // The accepted map (null until set_map() is called)
  shared_ptr<OccupancyGrid> grid_;
  int max_iterations_{numeric_limits<int>::max()};
  double timeout_seconds_{120.0};
};

} // namespace mission_level_route_planner
