#include "Dijkstras.cpp"
#include "grid.cpp"
#include <octomap/octomap.h>
#include <octomap/OcTree.h> // not sure if I "need" these, but if I do, I may also need to download and implement the header files for octomap (yay)
#include <octomap/point3d.h>


using namespace std;
namespace mission_level_route_planner{ // Using namespace to not get it confused
double findDistance(double first_x, double first_y, double second_x, double second_y){
    return sqrt(pow(second_x-first_x,2)+pow(second_y-first_y,2)); // Distance converter to figure out intial node point values
}

Index point3dToIndex(
    const OccupancyGrid& grid,
    const octomap::point3d& point)
{
    Waypoint position{
        point.x(),
        point.y(),
        point.z()
    };

    return grid.to_index(position);
}


// Find the shortest path around occupied cells
vector<octomap::point3d> Pathfinders::findPath(
    const OccupancyGrid& grid,
    const octomap::point3d& start,
    const octomap::point3d& destination)
{
    // Convert point3d coordinates to grid indices
    Index src = point3dToIndex(grid, start);
    Index dest = point3dToIndex(grid, destination);

vector<octomap::point3d> Pathfinders::findPath(
    const OccupancyGrid& grid,
    const octomap::point3d& start,
    const octomap::point3d& destination)
{
    // Convert point3d coordinates to grid indices
    Index src = point3dToIndex(grid, start);
    Index dest = point3dToIndex(grid, destination);

    // Make sure both points are safe
    if (!grid.is_free(src) || !grid.is_free(dest)) {
        return {};
    }

    // Assign each grid cell a numerical node ID
    map<Index, int> nodeIDs;

    // Store each node's grid coordinates
    vector<Index> nodes;

    // Adjacency list used by Dijkstra
    // pair = {neighbor ID, travel distance}
    vector<vector<pair<double, double>>> adj;

    // Queue for discovering possible routes
    queue<int> q;

    // Initialize starting node
    nodeIDs[src] = 0;
    nodes.push_back(src);
    adj.emplace_back();
    q.push(0);

    // Discover safe cells and build the graph
    while (!q.empty()) {

        int u = q.front();
        q.pop();

        // Get the current grid position
        Index current = nodes[u];

        // Find all safe neighbors using OccupancyGrid
        vector<Index> neighbors =
            grid.neighbors(current, 6);

        for (const Index& neighbor : neighbors) {

            int v;

            // Check whether the neighbor already exists
            auto it = nodeIDs.find(neighbor);

            if (it == nodeIDs.end()) {

                // Give the new cell its own node ID
                v = static_cast<int>(nodes.size());

                nodeIDs[neighbor] = v;
                nodes.push_back(neighbor);

                // Create an empty adjacency list for it
                adj.emplace_back();

                // Explore this cell later
                q.push(v);

            } else {
                v = it->second;
            }

            // Calculate the cost to move between cells
            double weight =
                grid.step_cost(current, neighbor);

            // Store the connection for Dijkstra
            adj[u].push_back({
                static_cast<double>(v),
                weight
            });
        }
    }

    // Check whether the destination was discovered
    if (nodeIDs.find(dest) == nodeIDs.end()) {
        return {};
    }

    // Find destination node ID
    int destID = nodeIDs[dest];

    // Use your EXISTING Dijkstra function
    vector<double> shortestPath =
        ::dijkstra(adj, 0.0, static_cast<double>(destID));

    // Check that a complete route was returned
    if (shortestPath.empty() ||
        shortestPath.front() != 0.0 ||
        shortestPath.back() != static_cast<double>(destID)) {
        return {};
    }

    // Convert the shortest path into OctoMap points
    vector<octomap::point3d> finalPath;

    for (double id : shortestPath) {

        Index cell = nodes[static_cast<int>(id)];

        Waypoint world = grid.to_world(cell);

        finalPath.emplace_back(
            static_cast<float>(world.x),
            static_cast<float>(world.y),
            static_cast<float>(world.z)
        );
    }
     int nextID = static_cast<int>(shortestPath[1]);

    Index nextIndex = nodes[nextID];

    Waypoint nextWorld = grid.to_world(nextIndex);

    octomap::point3d nextPoint(
        static_cast<float>(nextWorld.x),
        static_cast<float>(nextWorld.y),
        static_cast<float>(nextWorld.z) );
    return startPath(nextPoint, destination, grid);

}
vector<octomap::point3d> startpath(octomap::point3d orgin, octomap::point3d dest, OccupancyGrid grid){
    std::vector<octomap::point3d> ray;
    std::vector<octomap::point3d> path;
    std::vector<octomap::point3d> occu;
    std::vector<octomap::OcTreeNode*> occuNode;

    
    bool check = true;
    octomap::point3d prev(0.0,0.0,0.0)
    if (tree_.computeRay(orgin, dest, ray)) {
        for (size_t i = 0; i < ray.size(); i++) {
            octomap::point3d point = ray[i];
            octomap::OcTreeNode* node = tree_.search(point)
            if (check){ //prev is unoccupied
                if (node == nullptr){
                    std::cout << "Node failed";
                    return ray;
                }
                else if(tree_.isNodeOccupied(node)){//current is occupied
                    octomap::OcTreeNode* prevNode = tree_.search(prev)
                    occu.push_back(prev);
                    occuNode.push_back(prevNode);
                    occu.push_back(point);
                    occuNode.push_back(node);
                    check = false;
                }
                else { // current is occupied
                    prev = point;
                    path.push_back(point);
                }
            }
            else { //previous is occupied
                if(node == nullptr){
                    std::cout << "Node failed, but there is a current obstruction";
                    return ray;
                }
                else if(!(tree_.isNodeOccupied(node))){ // current is unoccupied
                    std::vector<octomap::point3d> route = findPath(grid, prev, point);
                    path.insert(prev.end(),route.beign(),route.end());
                    return path;
                }
                else { //current is occupied
                    occu.push_back(point);
                    occuNode.push_back(node);
                }
            }
        }
    }
    return 0;
}

vector<octomap::point3d> pathfinder(double orginX, double orginY, double destX, double destY, double staticZ){
    octomap::point3d orgin(orginX, orginY, staticZ); //Edit the Z axis to the set height
    octomap::point3d dest(destX, destY, staticZ); //Edit the Z axis to the set height
    return startpath(orgin,dest);
}

private:
shared_ptr<octomap::OcTree> tree_;

}//end namespace