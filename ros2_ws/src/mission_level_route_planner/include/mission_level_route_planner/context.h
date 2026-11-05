#pragma once

#include <memory>

#include "mission_level_route_planner/grid.h"
#include "mission_level_route_planner/pathfinding_strategy.h"

using namespace std;

namespace mission_level_route_planner
{

// Strategy pattern context. Validates start and goal, then hands off to the chosen algorithm.
class Context
{
public:
  Context(shared_ptr<OccupancyGrid> grid, shared_ptr<PathfindingStrategy> strategy);

  void set_strategy(shared_ptr<PathfindingStrategy> strategy);
  PlanStatus validate(const Index & start, const Index & goal) const;
  PlanResult plan(const Index & start, const Index & goal);

private:
  shared_ptr<OccupancyGrid> grid_;
  shared_ptr<PathfindingStrategy> strategy_;
};

}  // namespace mission_level_route_planner
