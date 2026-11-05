#pragma once

#include "mission_level_route_planner/models.h"

using namespace std;

namespace mission_level_route_planner
{

// Every algorithm (Dijkstra, A*, RRT) implements this one method.
class PathfindingStrategy
{
public:
  virtual ~PathfindingStrategy() = default;

  // Never throw for "no path": return a PlanResult with status NO_PATH.
  virtual PlanResult execute_pathfinding(const Index & start, const Index & goal) = 0;
};

}  // namespace mission_level_route_planner
