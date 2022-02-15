// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <cartesian_panda/cartesian_controller.h>

#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <iostream>
#include <unistd.h>
#include <termios.h>

#include <controller_interface/controller_base.h>
#include <franka_hw/franka_cartesian_command_interface.h>
#include <hardware_interface/hardware_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace cartesian_panda {

// Error checking ------------------------------------------------------------------------------
bool CartesianController::init(hardware_interface::RobotHW* robot_hardware,
                                          ros::NodeHandle& node_handle) {
  cartesian_pose_interface_ = robot_hardware->get<franka_hw::FrankaPoseCartesianInterface>();
  if (cartesian_pose_interface_ == nullptr) {
    ROS_ERROR(
        "CartesianController: Could not get Cartesian Pose "
        "interface from hardware");
    return false;
  }

  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("CartesianController: Could not get parameter arm_id");
    return false;
  }

  try {
    cartesian_pose_handle_ = std::make_unique<franka_hw::FrankaCartesianPoseHandle>(
        cartesian_pose_interface_->getHandle(arm_id + "_robot"));
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM(
        "CartesianController: Exception getting Cartesian handle: " << e.what());
    return false;
  }

  auto state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR("CartesianController: Could not get state interface from hardware");
    return false;
  }

  try {
    auto state_handle = state_interface->getHandle(arm_id + "_robot");

    std::array<double, 7> q_start{{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    for (size_t i = 0; i < q_start.size(); i++) {
      if (std::abs(state_handle.getRobotState().q_d[i] - q_start[i]) > 0.1) {
        ROS_ERROR_STREAM(
            "CartesianController: Robot is not in the expected starting position for "
            "running this example. Run `roslaunch franka_example_controllers move_to_start.launch "
            "robot_ip:=<robot-ip> load_gripper:=<has-attached-gripper>` first.");
        return false;
      }
    }
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM(
        "CartesianController: Exception getting state handle: " << e.what());
    return false;
  }

  return true;
}

char getch(void)
{
    char buf = 0;
    struct termios old = {0};
    fflush(stdout);
    if(tcgetattr(0, &old) < 0)
        perror("tcsetattr()");
    old.c_lflag &= ~ICANON;
    old.c_lflag &= ~ECHO;
    old.c_cc[VMIN] = 1;
    old.c_cc[VTIME] = 0;
    if(tcsetattr(0, TCSANOW, &old) < 0)
        perror("tcsetattr ICANON");
    if(read(0, &buf, 1) < 0)
        perror("read()");
    old.c_lflag |= ICANON;
    old.c_lflag |= ECHO;
    if(tcsetattr(0, TCSADRAIN, &old) < 0)
        perror("tcsetattr ~ICANON");
    //printf("%c\n", buf);
    return buf;
 }

// Control loop -------------------------------------------------------------------------------
void CartesianController::starting(const ros::Time& /* time */) {
  initial_pose_ = cartesian_pose_handle_->getRobotState().O_T_EE_d;
  elapsed_time_ = ros::Duration(0.0);
}

void CartesianController::update(const ros::Time& /* time */,
                                            const ros::Duration& period) {
  // Calculate new pose
  elapsed_time_ += period;

  double delta_x; // = 0;//speed / period.toSec();
  double delta_y; // = 0;//speed / period.toSec();

  std::cout << "Please enter x value: " << std::flush;
  std::cin >>  delta_x;
  std::cout << "Please enter y value: " << std::flush;
  std::cin >>  delta_y;
  
  std::cout << "delta_x = " << std::to_string(delta_x) << std::endl;
  std::cout << "delta_y = " << std::to_string(delta_y) << std::endl;


  // set new pose
  std::array<double, 16> new_pose = initial_pose_;
  new_pose[12] -= delta_x;
  new_pose[14] -= delta_y;
  std::cout << "Executing...";
  cartesian_pose_handle_->setCommand(new_pose);
}

}  // namespace cartesian_panda

PLUGINLIB_EXPORT_CLASS(cartesian_panda::CartesianController,
                       controller_interface::ControllerBase)
