#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <string>

using namespace std::chrono_literals;

class MissionRoutePlannerNode : public rclcpp::Node
{
  public:
    MissionRoutePlannerNode()
    : Node("Mission_Route_Planner")
    {

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

    //test to see if goal pose isn't empty during runtime
    bool testGoalPose() {
      for(double a : this->goal_pose) {
         RCLCPP_INFO(this->get_logger(), "Goal pose %f", a); 
      }
      if(this->goal_pose.size() != 0)
      {
        return true;
      }
      return false;
  }
    private:
      std::vector<double> goal_pose; 
      rclcpp::TimerBase::SharedPtr timer_;
  };


int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  rclcpp::init(argc, argv);
 
  auto node = std::make_shared<MissionRoutePlannerNode>();
  node->testGoalPose();
  std::vector<double> goalPose = node->getGoalPose(); //get goal pose
  rclcpp::spin(node); //<- Note: Runs timer_callback() above if we need to update goal during runtime
  rclcpp::shutdown();
  return 0;
}
