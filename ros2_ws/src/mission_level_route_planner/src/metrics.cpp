#include "mission_level_route_planner/metrics.h"


namespace mission_level_route_planner {

    // Calculate the total length of a 3D path in meters.
    double calculatePathLength(const std::vector<Index>& path, 
                                const OccupancyGrid& grid)
    {
        double totalLength = 0.0;

        // Go through each segment of the path and calculate the distance.
        for (std::size_t i = 1; i < path.size(); i++) {

            // Calculate the distance between consecutive points in the path.
            totalLength += grid.step_cost(path[i - 1], path[i]);
        }

        return totalLength;
    }
    
    // Calculate the elapsed computation time in seconds.
    double calculateComputationTime(const std::chrono::steady_clock::time_point& startTime,
                                    const std::chrono::steady_clock::time_point& endTime)
    {
        double elapsedTime = std::chrono::duration<double>
                            (endTime - startTime).count();

        return elapsedTime;
    }
   
}