#pragma once

#include <memory>

#include "mission_level_route_planner/grid.h"
#include "mission_level_route_planner/models.h"

namespace mission_level_route_planner
{

class MissionPlanner
{
public:
  void set_map(std::shared_ptr<OccupancyGrid> grid);

  bool has_map() const;
  std::shared_ptr<OccupancyGrid> map() const;

  PlanStatus validate(
    const Waypoint & start,
    const Waypoint & goal) const;

private:
  std::shared_ptr<OccupancyGrid> grid_;
};

}  // namespace mission_level_route_planner
