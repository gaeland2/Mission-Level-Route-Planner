
#pragma once

#include "grid.h"
#include "models.h"
#include <vector>

using namespace std;


namespace mission_level_route_planner {


//Runs a star search on given occupancy grid set in intializaiton from a constructot
class AStarSearch {
        

    public:
        //Constructs an AStarSearch object that takes in an occupancy grid which a* is calculated on 
        AStarSearch(OccupancyGrid occupancyGrid);
        //sets the occupancy grid to which the a* path is calculated on 
        void setOccupancyGrid(OccupancyGrid grid);
        //returns a list of waypoints that returns the shortest path to destinatin obstacle free
        vector<Waypoint> getAStarPath();

    private:
        OccupancyGrid grid;
        PlanResult plan; 
        //Gets the euclidian distance between three points 
        double getEuclideanDistance(Waypoint a, Waypoint b);
};  // namespace mission_level_route_planner

}
