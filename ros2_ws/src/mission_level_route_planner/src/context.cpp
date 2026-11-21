#include "mission_level_route_planner/context.h"
#include "mission_level_route_planner/path_smoother.h"

#include <utility>
#include <stdexcept>
#include <cmath>

using namespace std;

namespace mission_level_route_planner
{

Context::Context(
  shared_ptr<OccupancyGrid> grid, shared_ptr<PathfindingStrategy> strategy)
: grid_(std::move(grid)), strategy_(std::move(strategy))
{
}

void Context::set_strategy(shared_ptr<PathfindingStrategy> strategy)
{
  strategy_ = std::move(strategy);
}

PlanStatus Context::validate(const Index & start, const Index & goal) const
{
  if (!grid_->in_bounds(start) || !grid_->in_bounds(goal)) {return PlanStatus::OUT_OF_BOUNDS;}
  if (!grid_->is_free(start)) {return PlanStatus::START_BLOCKED;}
  if (!grid_->is_free(goal)) {return PlanStatus::GOAL_BLOCKED;}
  return PlanStatus::OK;
}

PlanResult Context::plan(const Index & start, const Index & goal, bool smooth_path)
{
  const PlanStatus status = validate(start, goal);
  if (status != PlanStatus::OK) {
    PlanResult r;
    r.status = status;
    return r;
  }

  PlanResult result = strategy_->execute_pathfinding(start, goal);

  if (smooth_path && result.success() && !result.path.empty()) {
    PathSmoother smoother(grid_);

    try {
      result.path = smoother.simplify(result.path);

      // Recalculate the length of the smoothed path.
      result.length_m = 0.0;

      for (size_t i = 1; i < result.path.size(); ++i) {
        const Index & a = result.path[i - 1];
        const Index & b = result.path[i];

        const double dx = static_cast<double>(b[0] - a[0]);
        const double dy = static_cast<double>(b[1] - a[1]);
        const double dz = static_cast<double>(b[2] - a[2]);

        result.length_m +=
          std::sqrt(dx * dx + dy * dy + dz * dz) * grid_->resolution();
      }

    } catch (const std::invalid_argument &) {
      // Do not return a potentially unsafe path.
      result.status = PlanStatus::NO_PATH;
      result.path.clear();
      result.length_m = 0.0;
    }
  }

  return result;
}

}  // namespace mission_level_route_planner
