#pragma once

#include <vector> 
#include <chrono>
#include "mission_level_route_planner/grid.h"

namespace mission_level_route_planner {

    // Calculate the total length of a 3D path in meters.
    double calculatePathLength(const std::vector<Index>& path, 
                                const OccupancyGrid& grid);
    
    // Calculate the elapsed computation time in seconds.
    double calculateComputationTime(const std::chrono::steady_clock::time_point& startTime,
                                    const std::chrono::steady_clock::time_point& endTime);

} 