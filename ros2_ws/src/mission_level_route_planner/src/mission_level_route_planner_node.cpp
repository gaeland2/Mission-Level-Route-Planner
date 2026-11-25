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

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/path.hpp>

#include "mission_level_route_planner/mission_planner.h"


using namespace std::chrono_literals;
using namespace mission_level_route_planner;

class MissionRoutePlannerNode : public rclcpp::Node
{
  public:
    MissionRoutePlannerNode()
    : Node("Mission_Route_Planner")
    {

      //declare octomap parameter
      _octomapSubscription = this->create_subscription<octomap_msgs::msg::Octomap>(
          "/octomap_binary",
          rclcpp::QoS(10),
          std::bind(&MissionRoutePlannerNode::octomapCallback, this, std::placeholders::_1));


      //Declaring paramater
      this->declare_parameter<std::vector<double>>
      ("start_pose", std::vector<double>{0.0, 0.0, 0.0});
      start_pose= this->get_parameter("start_pose").as_double_array();
      this->declare_parameter<double>("safety_margin", 0.0);
      this->declare_parameter<double>("extra_margin", 0.0);
      // Search guards for Dijkstra (see MissionPlanner::set_search_limits)
      this->declare_parameter<int>("max_iterations", 2147483647);
      this->declare_parameter<double>("planning_timeout_s", 120.0);

      //Declare rviz listener
      _initialPoseSubscription = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose", 10,
      std::bind(&MissionRoutePlannerNode::initialPoseCallback, this, std::placeholders::_1));

      _goalPoseSubscription = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", 10,
      std::bind(&MissionRoutePlannerNode::goalPoseCallback, this, std::placeholders::_1));

      _pathPublisher = create_publisher<nav_msgs::msg::Path>("/planned_path", 10);

      

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
    
    std::vector<double> getStartPose(){
      return this->start_pose;
    }


    std::shared_ptr<OccupancyGrid> getOccupancyGrid(std::shared_ptr<octomap::OcTree> _octree)
    {
      shared_ptr<octomap::OcTree> tree = _octree;
      
      double min_x, min_y, min_z, max_x, max_y, max_z;
      tree->getMetricMin(min_x, min_y, min_z);
      tree->getMetricMax(max_x, max_y, max_z);
      
      const double res = tree->getResolution();
      const int nx = static_cast<int>(std::ceil((max_x - min_x) / res));
      const int ny = static_cast<int>(std::ceil((max_y - min_y) / res));
      const int nz = static_cast<int>(std::ceil((max_z - min_z) / res));

      auto grid = std::make_shared<OccupancyGrid>(
        nx, ny, nz, res, Waypoint{min_x, min_y, min_z});

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

      rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr _initialPoseSubscription;
      rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr _goalPoseSubscription;
      rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr _pathPublisher;

      /// Paramater variables, i.e goal_pose, start pose
      std::vector<double> start_pose; 
      std::vector<double> goal_pose; 
      rclcpp::TimerBase::SharedPtr timer_;

      //Octree variables
      std::shared_ptr<octomap::OcTree> _octree;
      std::mutex _octreeMutex;
      std::string map_frame_id_;

      //OccupancyGrid, PathResult, etc
      std::shared_ptr<OccupancyGrid> grid;
      //<optional> meaning grid can be null at start if octomap is not available


    void initialPoseCallback(
      const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
      {
        const auto & p = msg->pose.pose.position;
        //const auto & q = msg->pose.pose.orientation;

        const double safety_margin = this->get_parameter("safety_margin").as_double();
        const double extra_margin = this->get_parameter("extra_margin").as_double();
        const double offset_above_ground = safety_margin + extra_margin;
        const double raised_z = p.z + offset_above_ground; // Raise z to prevent the safety margin from overlapping with the ground

        start_pose = {p.x, p.y, raised_z};
        this->set_parameter(rclcpp::Parameter("start_pose", start_pose));
        RCLCPP_INFO(
          rclcpp::get_logger("rclcpp"),
          "initial pose (subscribed from rviz): %.2f %.2f %.2f set",
          p.x, p.y, raised_z);

        // The grid only depends on the octomap, so don't rebuild it here.
        publishPath();
      }

    void goalPoseCallback(
      const geometry_msgs::msg::PoseStamped::SharedPtr msg)
      {
        const auto & p = msg->pose.position; //pos
        //const auto & q = msg->pose.orientation; //orientaiton

        const double safety_margin = this->get_parameter("safety_margin").as_double();
        const double extra_margin = this->get_parameter("extra_margin").as_double();
        const double offset_above_ground = safety_margin + extra_margin;
        const double raised_z = p.z + offset_above_ground; // Raise z to prevent the safety margin from overlapping with the ground

        goal_pose = {p.x, p.y, raised_z};
        RCLCPP_INFO(
          rclcpp::get_logger("rclcpp"),
          "goal pose (subscribed from rviz): %.2f %.2f %.2f set",
          p.x, p.y, raised_z);
        publishPath();
      }

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
        map_frame_id_ = msg->header.frame_id;

        updateGridAndPublishPath();
    }

    void updateGridAndPublishPath()
    {
      if (!_octree) {
        return;
      }
      if (start_pose.size() != 3) {
        RCLCPP_WARN(get_logger(), "Cannot build occupancy grid: start_pose must contain x, y, z");
        return;
      }

      grid = getOccupancyGrid(_octree);
      grid->set_safety_margin(this->get_parameter("safety_margin").as_double());
      RCLCPP_INFO(get_logger(), "OccupancyGrid built from OctoMap.");
      publishPath();
    }

    void publishPath()
    {
      if (!grid || start_pose.size() != 3 || goal_pose.size() != 3) {
        return;
      }

      const PlanRequest request{
        Waypoint{start_pose[0], start_pose[1], start_pose[2]},
        Waypoint{goal_pose[0], goal_pose[1], goal_pose[2]}};
      MissionPlanner planner(grid);
      planner.set_search_limits(
        static_cast<int>(this->get_parameter("max_iterations").as_int()),
        this->get_parameter("planning_timeout_s").as_double());
      const PlanStatus status = planner.validate(request);

      nav_msgs::msg::Path path;
      path.header.stamp = now();
      path.header.frame_id = map_frame_id_;

      if (status == PlanStatus::OK) {
        const PlanResult result = planner.plan(request);
        RCLCPP_INFO(
          get_logger(), "Plan %s: %d nodes expanded in %.2f s",
          mission_level_route_planner::to_string(result.status),
          result.nodes_expanded, result.compute_time_s);
        if (!result.success() || result.path.empty()) {
          RCLCPP_WARN(
            get_logger(), "No path published (status %s).",
            mission_level_route_planner::to_string(result.status));
        } else {
          path.poses.reserve(result.path.size());
          for (const Index & index : result.path) {
            const Waypoint waypoint = grid->to_world(index);
            geometry_msgs::msg::PoseStamped pose;
            pose.header = path.header;
            pose.pose.position.x = waypoint.x;
            pose.pose.position.y = waypoint.y;
            pose.pose.position.z = waypoint.z;
            pose.pose.orientation.w = 1.0;
            path.poses.push_back(std::move(pose));
          }
        }
      } else {
        RCLCPP_WARN(
          get_logger(), "Cannot publish route: planner status is %s",
          mission_level_route_planner::to_string(status));
      }

      _pathPublisher->publish(path);
      if (!path.poses.empty()) {
        RCLCPP_INFO(
          get_logger(), "Path successfully generated with %zu poses published.",
          path.poses.size());
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
