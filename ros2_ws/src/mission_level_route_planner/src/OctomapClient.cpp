#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <octomap/octomap.h>
#include <octomap_msgs/srv/get_octomap.hpp>
#include "rclcpp/rclcpp.hpp"
#include <octomap_msgs/conversions.h>
#include <memory.h>

using namespace std;
using namespace std::chrono_literals;
using namespace octomap;


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
