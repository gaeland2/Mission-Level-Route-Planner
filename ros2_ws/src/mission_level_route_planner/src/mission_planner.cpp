#include "mission_level_route_planner/mission_planner.h"

#include <cmath>
#include <stdexcept>
#include <utility>

using namespace std;

namespace mission_level_route_planner
{

namespace
{
bool is_finite(const Waypoint & p)
{
  return isfinite(p.x) && isfinite(p.y) && isfinite(p.z);
}
} 

// Stores map, throws invalid_argument if grid is null
void MissionPlanner::set_map(shared_ptr<OccupancyGrid> grid)
{
  if (!grid) {throw invalid_argument("map must not be null");}
  grid_ = move(grid);
}

// Check both start & goal poses against the map and return a corresponding PlanStatus
PlanStatus MissionPlanner::validate(const Waypoint & start, const Waypoint & goal) const
{
  if (!grid_) {return PlanStatus::NO_MAP;}
  if (!is_finite(start) || !is_finite(goal)) {return PlanStatus::OUT_OF_BOUNDS;}

  const Index s = grid_->to_index(start);
  const Index g = grid_->to_index(goal);
  if (!grid_->in_bounds(s) || !grid_->in_bounds(g)) {return PlanStatus::OUT_OF_BOUNDS;}
  if (!grid_->is_free(s)) {return PlanStatus::START_BLOCKED;}
  if (!grid_->is_free(g)) {return PlanStatus::GOAL_BLOCKED;}
  return PlanStatus::OK;
}

}  
