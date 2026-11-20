#include "mission_level_route_planner/mission_planner.h"
#include "mission_level_route_planner/grid.h"
#include <cmath>
#include <stdexcept>
#include <utility>
#include "mission_level_route_planner/dijkstra.h"

using namespace std;

namespace mission_level_route_planner
{


MissionPlanner::MissionPlanner(shared_ptr<OccupancyGrid> grid)
{
  if (!grid) {throw invalid_argument("map must not be null");}
  set_map(std::move(grid));
}


namespace
{
bool is_finite(const Waypoint & p)
{
  return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
}
}  // namespace

// Stores map, throws invalid_argument if grid is null
void MissionPlanner::set_map(shared_ptr<OccupancyGrid> grid)
{
  if (!grid) {throw invalid_argument("map must not be null");}
  grid_ = move(grid);
}


//Gets a list of waypoints to push to mavros
// Create a plan request struct by inputting the start coordinates and goal coordinates
// ie. PlanRequest req;
// req.start = (must convert our vector of doubles to waypoint structure)
// req.end = (like above)

vector<Waypoint> MissionPlanner::getMissionPath(const PlanRequest & request) const
{
  if (!grid_) {throw invalid_argument("map must not be null");}

  if (validate(request) != PlanStatus::OK) {return {};}

  const Index startIndex = grid_->to_index(request.startPose);
  const Index endIndex = grid_->to_index(request.endPose);

  Dijkstra djk(grid_);
  const PlanResult result = djk.execute_pathfinding(startIndex, endIndex);
  if (!result.success()) {return {};}

  vector<Waypoint> wp;
  wp.reserve(result.path.size());
  for (const Index & index : result.path) {
    wp.push_back(grid_->to_world(index));
  }
  return wp;
}

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
