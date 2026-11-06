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
#include "mission_level_route_planner/grid.h"
#include <optional>


using namespace std::chrono_literals;
using namespace mission_level_route_planner;

class MissionRoutePlannerNode : public rclcpp::Node
{
  public:
    MissionRoutePlannerNode()
    : Node("Mission_Route_Planner")
    {
      _octomapSubscription = this->create_subscription<octomap_msgs::msg::Octomap>(
          "/octomap_binary",
          rclcpp::QoS(10),
          std::bind(&MissionRoutePlannerNode::octomapCallback, this, std::placeholders::_1));


      //Declaring paramater
      this->declare_parameter<std::vector<double>>
      ("start_pose", std::vector<double>{0.0, 0.0, 0.0});
      start_pose= this->get_parameter("start_pose").as_double_array();
      //
      
      timer_ = this->create_wall_timer(
      1000ms, std::bind(&MissionRoutePlannerNode::timer_callback, this));
      
    }


    //For callback (Node execute this function below every 1000ms)
    // and interacting and updating parameters during runtime activity,
    // node always run in background, do not delete
    void timer_callback()
    {
      start_pose = this->get_parameter("start_pose").as_double_array();
      std::vector<rclcpp::Parameter> all_new_parameters{rclcpp::Parameter("start_pose", start_pose)};
      //RCLCPP_INFO(this->get_logger(), "Running Node ");

      this->set_parameters(all_new_parameters);
    }
    
    std::vector<double> getGoalPose(){
      return this->start_pose;
    }


    std::shared_ptr<OccupancyGrid> getOccupancyGrid(Waypoint origin, std::shared_ptr<octomap::OcTree> _octree)
    {
      shared_ptr<octomap::OcTree> tree = _octree;
      
      double min_x, min_y, min_z, max_x, max_y, max_z;
      tree->getMetricMin(min_x, min_y, min_z);
      tree->getMetricMax(max_x, max_y, max_z);
      
      const double res = tree->getResolution();
      const int nx = static_cast<int>(std::ceil((max_x - min_x) / res));
      const int ny = static_cast<int>(std::ceil((max_y - min_y) / res));
      const int nz = static_cast<int>(std::ceil((max_z - min_z) / res));

      auto grid = std::make_shared<OccupancyGrid>(nx, ny, nz, res, origin);

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
            grid->block(Index{i, j, k});
      }
      return grid;
    }


    
    private:

      // Callbacks/subscriptions/clients/etc
      rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr _octomapSubscription;
      rclcpp::Client<octomap_msgs::srv::GetOctomap>::SharedPtr _client;

      /// Paramater variables, i.e goal_pose, start pose
      std::vector<double> start_pose; 
      rclcpp::TimerBase::SharedPtr timer_;

      //Octree variables
      std::shared_ptr<octomap::OcTree> _octree;
      std::mutex _octreeMutex;

      //OccupancyGrid, PathResult, etc
      std::shared_ptr<OccupancyGrid> grid;
      //<optional> meaning grid can be null at start if octomap is not available

      
    void octomapCallback(
        const octomap_msgs::msg::Octomap::SharedPtr msg)
    {
        octomap::AbstractOcTree* abstract = octomap_msgs::msgToMap(*msg);
        if (!abstract) {
            RCLCPP_WARN(get_logger(), "Failed to convert Octomap message");
            return;
        }

        auto* tree = dynamic_cast<octomap::OcTree*>(abstract);
        if (!tree) {
            RCLCPP_WARN(get_logger(), "Message is not an OcTree");
            delete abstract;
            return;
        }
        else if (tree) {
          RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Octomap Initialized....");
        }


        //below is for updating the tree if we change the tree during runtime...
        //avoids race conditions
        std::lock_guard<std::mutex> lock(_octreeMutex);
        _octree.reset(tree);   // takes ownership, frees the previous tree

        //BUILDS GRID HERE.
        grid = this->getOccupancyGrid(Waypoint{start_pose[0], start_pose[1], start_pose[2]}, _octree);
        if(!grid)
        {
           RCLCPP_WARN(get_logger(), "grid is not built");
        }
        else
        {
          RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "OccupancyGrid is built, pointed by grid variable.");
        }


    }
  };



  //Everything goes here once we connect everything
  //->get list of points from goal pose to end pose ->publish path to mavros
int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;


  rclcpp::init(argc, argv);
  auto node = std::make_shared<MissionRoutePlannerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown(); 
  return 0;
}
