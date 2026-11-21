#pragma once

#include <memory>
#include <vector>

#include "mission_level_route_planner/grid.h"
#include "mission_level_route_planner/models.h"

namespace mission_level_route_planner
{

class PathSmoother
{
public:
  explicit PathSmoother(std::shared_ptr<OccupancyGrid> grid);

  // Remove unnecessary waypoints while keeping the path collision-free.
  // The length tolerance limits how much longer the result may be.
  std::vector<Index> simplify(
    const std::vector<Index> & path,
    double length_tolerance = 0.05) const;

private:
  std::shared_ptr<OccupancyGrid> grid_;

  // Check whether a straight segment is collision-free.
  bool line_of_sight(const Index & start, const Index & end) const;

  // Calculate the total path length in meters.
  double path_length(const std::vector<Index> & path) const;
};

}  // namespace mission_level_route_planner