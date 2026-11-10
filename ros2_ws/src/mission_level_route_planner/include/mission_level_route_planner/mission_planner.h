#pragma once

#include <memory>

#include "mission_level_route_planner/grid.h"
#include "mission_level_route_planner/models.h"

using namespace std;

namespace mission_level_route_planner
{

{
  Waypoint start;
  Waypoint goal;
};

class MissionPlanner
{
public:
  // Accept a map. Throws invalid_argument if grid is null
  void set_map(shared_ptr<OccupancyGrid> grid);
  
  bool has_map() const {return grid_ != nullptr;}
  shared_ptr<OccupancyGrid> map() const {return grid_;}

  // Check both start & goal poses against the map and return a PlanStatus
  PlanStatus validate(const Waypoint & start, const Waypoint & goal) const;
  // Same check, just taking a PlanRequest instead
  PlanStatus validate(const PlanRequest & request) const
  {
    return validate(request.start, request.goal);
  }

private:
  // The accepted map (null until set_map() is called)
  shared_ptr<OccupancyGrid> grid_;
};

} // namespace mission_level_route_planner