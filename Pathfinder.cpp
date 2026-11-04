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


vector<octomap::point3d> findPath(vector<octomap::OcTreeNode*> pathNode,vector<octomap::point3d> path, octomap::point3d dest) {
  /* This needs to have a few things:
  1. Need to make an area where I can "see" the object that is blocking (use)
  2. Needs to find a pathway around said object (mabye I use map, just shrink it to X bounds(find out X bonds))
  3. Need to compare distance from orgin (before object blocks) to distance to edge of object. Mabye?? 
  check out b-spline smoothing for pathing*/





}

vector<octomap::point3d> startpath(octomap::point3d orgin, octomap::point3d dest){
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
                    std::vector<octomap::point3d> route = findPath();
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

vector<octomap::point3d> pathfinderx`(double orginX, double orginY, double destX, double destY, double staticZ){
    octomap::point3d orgin(orginX, orginY, staticZ); //Edit the Z axis to the set height
    octomap::point3d dest(destX, destY, staticZ); //Edit the Z axis to the set height
    return startpath(orgin,dest);
}

private:
shared_ptr<octomap::OcTree> tree_;

}//end namespace