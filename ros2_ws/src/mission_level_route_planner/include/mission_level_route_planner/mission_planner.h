#pragma once

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

private:
  // The accepted map (null until set_map() is called)
  shared_ptr<OccupancyGrid> grid_;
};

} // namespace mission_level_route_planner
