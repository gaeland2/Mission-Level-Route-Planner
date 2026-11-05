#include "mission_level_route_planner/context.h"

#include <utility>

using namespace std;

namespace mission_level_route_planner
{

Context::Context(
  shared_ptr<OccupancyGrid> grid, shared_ptr<PathfindingStrategy> strategy)
: grid_(move(grid)), strategy_(move(strategy))
{
}

void Context::set_strategy(shared_ptr<PathfindingStrategy> strategy)
{
  strategy_ = move(strategy);
}

PlanStatus Context::validate(const Index & start, const Index & goal) const
{
  if (!grid_->in_bounds(start) || !grid_->in_bounds(goal)) {return PlanStatus::OUT_OF_BOUNDS;}
  if (!grid_->is_free(start)) {return PlanStatus::START_BLOCKED;}
  if (!grid_->is_free(goal)) {return PlanStatus::GOAL_BLOCKED;}
  return PlanStatus::OK;
}

PlanResult Context::plan(const Index & start, const Index & goal)
{
  const PlanStatus status = validate(start, goal);
  if (status != PlanStatus::OK) {
    PlanResult r;
    r.status = status;
    return r;
  }
  return strategy_->execute_pathfinding(start, goal);
}

}  // namespace mission_level_route_planner
