#include "mission_level_route_planner/astar.h"
#include "mission_level_route_planner/grid.h"
#include <set>
#include "queue"
#include <stdexcept>
#include <model.h>
#include <bits/stdc++.h>

using namespace std;
using mission_level_route_planner::Index;
using Entry = std::pair<double, int>; //Cell id maps to integer->

namespace mission_level_route_planner{
    class AStarSearch {
    public:
        //Constructs an AStarSearch object that takes in an occupancy grid which a* is calculated on 
        AStarSearch(OccupancyGrid occupancyGrid)
            : grid(occupancyGrid)
        {
        }

        //sets the occupancy grid to which the a* path is calculated on 
        void setOccupancyGrid(OccupancyGrid grid)
        {
            this->grid = grid;
        }


        PlanStatus getPlanStatus()
        {
            return this->plan.status;
        }

        //returns a list of Index that returns the shortest path to destinatin obstacle free,
        //Note this is the unconverted list of waypoints, must use to_waypoints() for Ros compatable
        //waypoitns
        vector<Index> getAStarPath(const Index & start, const Index & goal,
                                        int connectivity = 26)
        {

           priority_queue<Index> openNodes = {};
           set<Index> visited = {};
           vector<Waypoint> path;
           
           if(!grid.in_bounds(start) || !grid.in_bounds(goal)){
                plan.status = PlanStatus::OUT_OF_BOUNDS;
                return;}
           if (grid.is_occupied(start)){
                plan.status = PlanStatus::START_BLOCKED;
                return;}
           if (grid.is_occupied(goal)){
                plan.status = PlanStatus::GOAL_BLOCKED;
                return;}

           const int start_id = grid.to_id(start);
           const int goal_id = grid.to_id(goal);
           const int n = grid.cell_count();
           vector<double> g = {};
           vector<int> parent;
           vector<char> closed;
           vector<Index> resultPath;

           // the smallest heuristic value plus g value being popped
           priority_queue<Entry, vector<Entry>, greater<Entry>> open; 

           g[start_id] = 0.0;
           open.emplace(grid.step_cost(start,goal), start_id);
           while(!openNodes.empty())
           {
                const int cur_id = open.top().second; //pops id of the node that was picked
                open.pop();

                if(closed[cur_id]) {continue;}
                closed[cur_id] = 1;
                //

                //goal node found
                if (cur_id == goal_id)
                {
                    //backtrack
                    for (int id = goal_id; id != -1; id = parent[id]){
                        Index cellId = grid.from_id(id);
                        resultPath.push_back(cellId);
                    }
                    reverse(resultPath.begin(), resultPath.end());
                    plan.status = PlanStatus::OK;
                    return resultPath;
                }

                const Index current = grid.from_id(cur_id);
                //iterate through every neighbor, find the lowest cost
                for (const Index & neighbor : grid.neighbors(current,connectivity)) {
                    const int neighbor_id = grid.to_id(neighbor);
                    if(closed[neighbor_id]) {continue;} //if already closed ignore 

                    const double fCost = g[cur_id] + grid.step_cost(current, neighbor);
                    if(fCost < g[neighbor_id]) {
                        g[neighbor_id] = fCost;
                        parent[neighbor_id] = cur_id;
                        open.emplace(fCost + grid.step_cost(neighbor, goal), neighbor_id);
                    }
                }
           }

           //Plan failed 
           plan.status = PlanStatus::NO_PATH;
        }

    //returns a vector of waypoints to be pushed into mavros msgs 
    std::vector<Waypoint> to_waypoints(const std::vector<Index> & path) const
    {
    std::vector<Waypoint> out;
    out.reserve(path.size());
    for (const Index & i : path) {out.push_back(grid.to_world(i));}
    return out;
    }

        

    private:
        OccupancyGrid grid;
        PlanResult plan; 
        //Gets the euclidian distance between three points 
    };  // namespace mission_level_route_planner


}