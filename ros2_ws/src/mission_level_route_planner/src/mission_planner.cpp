#include "mission_level_route_planner/mission_planner.h"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace mission_level_route_planner
{

namespace
{

bool is_finite(const Waypoint & point)
{
  return std::isfinite(point.x) &&
         std::isfinite(point.y) &&
         std::isfinite(point.z);
}

}  // namespace

void MissionPlanner::set_map(std::shared_ptr<OccupancyGrid> grid)
{
  if (!grid) {
    throw std::invalid_argument("map must not be null");
  }

  grid_ = std::move(grid);
}

bool MissionPlanner::has_map() const
{
  return grid_ != nullptr;
}

std::shared_ptr<OccupancyGrid> MissionPlanner::map() const
{
  return grid_;
}

PlanStatus MissionPlanner::validate(
  const Waypoint & start,
  const Waypoint & goal) const
{
  if (!grid_) {
    return PlanStatus::NO_MAP;
  }

  // Check before converting because converting NaN or infinity to int is unsafe.
  if (!is_finite(start) || !is_finite(goal)) {
    return PlanStatus::INVALID_POSE;
  }

  const Index start_index = grid_->to_index(start);
  const Index goal_index = grid_->to_index(goal);

  if (!grid_->in_bounds(start_index) ||
      !grid_->in_bounds(goal_index)) {
    return PlanStatus::OUT_OF_BOUNDS;
  }

  if (!grid_->is_free(start_index)) {
    return PlanStatus::START_BLOCKED;
  }

  if (!grid_->is_free(goal_index)) {
    return PlanStatus::GOAL_BLOCKED;
  }

  return PlanStatus::OK;
}

}  // namespace mission_level_route_planner
