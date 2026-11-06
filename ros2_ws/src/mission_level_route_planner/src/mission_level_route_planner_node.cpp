#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <string>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <octomap/octomap.h>
#include <octomap_msgs/srv/get_octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <memory.h>
#include "mission_level_route_planner/grid.h"


using namespace std::chrono_literals;
using namespace mission_level_route_planner;

class MissionRoutePlannerNode : public rclcpp::Node
{
  public:
    MissionRoutePlannerNode()
    : Node("Mission_Route_Planner")
    {
      //Create client to request octomap
      _client = create_client<octomap_msgs::srv::GetOctomap>("octomap_binary");
      //Declaring paramater
      this->declare_parameter<std::vector<double>>
      ("goal_pose", std::vector<double>{1.0, 1.0, 1.0});
      goal_pose = this->get_parameter("goal_pose").as_double_array();
      //
      timer_ = this->create_wall_timer(
      1000ms, std::bind(&MissionRoutePlannerNode::timer_callback, this));
      
    }

    //For callback (Node execute this function below every 1000ms)
    // and interacting and updating parameters during runtime activity,
    // node always run in background, do not delete
    void timer_callback()
    {
      goal_pose = this->get_parameter("goal_pose").as_double_array();
      std::vector<rclcpp::Parameter> all_new_parameters{rclcpp::Parameter("goal_pose", goal_pose)};
      RCLCPP_INFO(this->get_logger(), "Running Node ");

      this->set_parameters(all_new_parameters);
    }
    
    std::vector<double> getGoalPose(){
      return this->goal_pose;
    }

    //Request octomap
    bool getOctomap() 
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


    
    private:
      rclcpp::Client<octomap_msgs::srv::GetOctomap>::SharedPtr _client;
      octomap::AbstractOcTree *abstractOcTree;
      octomap::OcTree *ocTree;
    OccupancyGrid OccupancyGrid;
      shared_ptr<octomap::OcTree> tree_;
      std::vector<double> goal_pose; 
      rclcpp::TimerBase::SharedPtr timer_;
  };



  //Everything goes here once we connect everything
  //->get list of points from goal pose to end pose ->publish path to mavros
int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;




  rclcpp::init(argc, argv);
 
  return 0;
}
