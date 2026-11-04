#include "mission_level_route_planner/astar.h"
#include "mission_level_route_planner/grid.h"
#include "queue"


using namespace std;

namespace mission_level_route_planner{
    class AStarSearch {
    public:
        //Constructs an AStarSearch object that takes in an occupancy grid which a* is calculated on 
        AStarSearch(const OccupancyGrid & grid, const Index & start,
                    const Index & goal, int connectivity = 26)
            : grid(occupancyGrid)
        {
            if(!grid.is_free(start) || !grid.is_free(goal)) return {}; 
            //throw error here
        }

        //sets the occupancy grid to which the a* path is calculated on 
        void setOccupancyGrid(OccupancyGrid grid);
        //returns a list of waypoints that returns the shortest path to destinatin obstacle free
        vector<Waypoint> getAStarPath();
        {

        }

    private:
        OccupancyGrid grid;
        PlanResult plan; 
        //Gets the euclidian distance between three points 
        double getEuclideanDistance(Waypoint a, Waypoint b);
    };  // namespace mission_level_route_planner


}