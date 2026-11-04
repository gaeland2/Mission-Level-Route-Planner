#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <octomap/octomap.h>
#include <octomap_msgs/srv/get_octomap.hpp>
#include "rclcpp/rclcpp.hpp"
#include <octomap_msgs/conversions.h>
#include <memory.h>
#include "mission_level_route_planner/grid.h"

using namespace std;
using namespace std::chrono_literals;
using namespace octomap;
using namespace mission_level_route_planner;

//Class to request Octree from running octomap_server.  
class OctomapClient : public rclcpp::Node
{
  public:
  OctomapClient() 
  : Node("octomap_client")
  {
    _client = create_client<octomap_msgs::srv::GetOctomap>("octomap_binary");
    requestMap();
  }


  //Returns a shared address to the octomap tree
  shared_ptr<octomap::OcTree> getOcTree() const{
    return tree_;
  }

  bool mapReady() const {return tree_ != nullptr; }

  //tests to see if given point is occupied or not
  bool testTree(){
    octomap::OcTreeNode *n = tree_->search(1.0, 2.0, 1.0);
    if(n == NULL)
    {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Unknown space");
    }
    bool isOccupied = n && tree_->isNodeOccupied(n);
    return isOccupied;
  }

  //Converts given octree requested from client to an occupancy grid used for
  //pathfinding, based on origin or start goal
  OccupancyGrid getOccupancyGrid(Waypoint origin)
  {
    shared_ptr<octomap::OcTree> tree = this->tree_;
    
    double min_x, min_y, min_z, max_x, max_y, max_z;
    tree->getMetricMin(min_x, min_y, min_z);
    tree->getMetricMax(max_x, max_y, max_z);

    const double res = tree->getResolution();
    const int nx = static_cast<int>(std::ceil((max_x - min_x) / res));
    const int ny = static_cast<int>(std::ceil((max_y - min_y) / res));
    const int nz = static_cast<int>(std::ceil((max_z - min_z) / res));

    OccupancyGrid grid = OccupancyGrid(nx, ny, nz, res, origin);

    //being filling occupancy grid
    for(auto it = tree->begin_leafs(), end = tree->end_leafs(); it !=end; ++it)
    {
      if (!tree->isNodeOccupied(*it)) continue;

      const double half = it.getSize() / 2.0;
        const int i0 = (int)std::floor((it.getX() - half - min_x) / res + 1e-6);
        const int i1 = (int)std::floor((it.getX() + half - min_x) / res - 1e-6);
        const int j0 = (int)std::floor((it.getY() - half - min_y) / res + 1e-6);
        const int j1 = (int)std::floor((it.getY() + half - min_y) / res - 1e-6);
        const int k0 = (int)std::floor((it.getZ() - half - min_z) / res + 1e-6);
        const int k1 = (int)std::floor((it.getZ() + half - min_z) / res + 1e-6);

     for (int i = i0; i <= i1; ++i)
      for (int j = j0; j <= j1; ++j)
        for (int k = k0; k <= k1; ++k)
          grid.block(Index{i, j, k});
    }
    return grid;
  }

  //converts given octree called from server

  private:
    rclcpp::Client<octomap_msgs::srv::GetOctomap>::SharedPtr _client;
    octomap::AbstractOcTree *abstractOcTree;
    octomap::OcTree *ocTree;
    shared_ptr<octomap::OcTree> tree_;

    //Requests octomap OctTree from octomap_server
    //Returns True if request is successful
    //Returns false if it fails.
    bool requestMap() 
    {
      auto request = std::make_shared<octomap_msgs::srv::GetOctomap::Request>();

      //check if service is available
      while(!_client->wait_for_service(3s)) {
        if(!rclcpp::ok()) {
          RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
          return false;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "/octomap_binary service not available, waiting again...");
      }

      //Find if service is ready, assign tree to shared pointer.
      if(_client->service_is_ready()) {
        _client->async_send_request(
          request, [this](rclcpp::Client<octomap_msgs::srv::GetOctomap>::SharedFuture future){
              auto response = future.get();
              RCLCPP_INFO(get_logger(), "Received octomap from /octomap_server, /octomap_binary: octomap_msgs/srv/GetOctomap");
              abstractOcTree = octomap_msgs::binaryMsgToMap(response->map);
              if(!abstractOcTree)
              {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "failed to convert binary OcTree");
                return;
              }
              ocTree = dynamic_cast<octomap::OcTree *>(abstractOcTree);
              if(!ocTree) {
                delete abstractOcTree; //wrong type 
                return;
                }
               tree_.reset(ocTree);
          });
      } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclpcpp"), "Failed to call /octomap_binary");
        return false;
      } 
      return true;
    }
      
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OctomapClient>();

  // Spin until the service response has been processed
  while (rclcpp::ok() && !node->mapReady()) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(50ms);   // avoid busy-waiting (#include <thread>)
  }

  if (node->mapReady()) {
    bool occupied = node->testTree();
    RCLCPP_INFO(node->get_logger(), "Point occupied: %s", occupied ? "yes" : "no");
  }

  rclcpp::shutdown();
  return 0;
}
