#include <cmath>
#include <math.h>
#include <array>
#include <string>
#include <vector>
#include <iostream>

#include <ros/ros.h>
#include <ros/topic.h>
#include <ros/node_handle.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <franka_gripper/GraspAction.h>
#include <franka_gripper/MoveAction.h>
#include <std_msgs/Float64MultiArray.h>

using franka_gripper::GraspAction;
using franka_gripper::MoveAction;
using GraspClient = actionlib::SimpleActionClient<GraspAction>;
using MoveClient = actionlib::SimpleActionClient<MoveAction>;

namespace cartesian_panda
{
static const int NUM_SPINNERS = 1;

  class Gripper
  {    
  public:

    Gripper() : spinner_(NUM_SPINNERS), grasp_client_("franka_gripper/grasp", true), move_client_("franka_gripper/move", true)
    {
      grip_sub_ = n_.subscribe<std_msgs::Float64MultiArray>("/unity/hand_grip", 1, &Gripper::GripCallback, this);

      spinner_.start();
      ros::waitForShutdown();
    };

  private:
    void GripCallback(const std_msgs::Float64MultiArrayConstPtr &msg)
    { 
      grasp_client_.waitForServer();
      move_client_.waitForServer();
      command_ = msg->data;
      double grip = command_[0]+command_[1];
      //ROS_INFO("Goal Send: %f", command_[0]+command_[1]);

      franka_gripper::GraspGoal graspgoal;
      franka_gripper::MoveGoal movegoal;

      graspgoal.speed = 1;
      graspgoal.width = 0;
      graspgoal.force = 100;
      graspgoal.epsilon.inner = 0.1;
      graspgoal.epsilon.outer = 0.1;      
      
      movegoal.speed = graspgoal.speed;
      movegoal.width = 0.04;

      if (grip > 0.075) {new_command_ = "open";}
      if (grip < 0.005) {new_command_ = "close";}    
      if (new_command_ != old_command_)
        {
          if (new_command_ == "open") { 
            move_client_.sendGoal(movegoal); 
            old_command_ = new_command_ ;
            move_client_.waitForResult();
            //ROS_INFO("open");
          }
          else if (new_command_ == "close") 
            { 
              grasp_client_.sendGoal(graspgoal); 
              old_command_ = new_command_ ;
              grasp_client_.waitForResult();
              //ROS_INFO("Close");
            }
        }      
    }

    ros::NodeHandle n_;
    ros::Subscriber grip_sub_;
    ros::AsyncSpinner spinner_;
    std::vector<double> command_;
    GraspClient grasp_client_;
    MoveClient move_client_;
    std::string old_command_;
    std::string new_command_;
  };
}  // namespace

int main (int argc, char **argv)
{
  ros::init(argc, argv, "gripper");
  cartesian_panda::Gripper grip;

  return 0;

}
