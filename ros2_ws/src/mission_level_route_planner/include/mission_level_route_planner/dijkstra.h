#pragma once

#include <memory>

#include "mission_level_route_planner/grid.h"
#include "mission_level_route_planner/pathfinding_strategy.h"

using namespace std;

namespace mission_level_route_planner
{

class Dijkstra : public PathfindingStrategy
{
public:
  // The grid is shared with the Context, so map updates are visible to the planner.
  explicit Dijkstra(shared_ptr<OccupancyGrid> grid, int connectivity = 26);

  PlanResult execute_pathfinding(const Index & start, const Index & goal) override;

private:
  shared_ptr<OccupancyGrid> grid_;
  int connectivity_;
};

}  // namespace mission_level_route_planner
