#!/usr/bin/env bash
cd
cd tele_panda_ws
source devel/setup.bash

rostopic pub -r 100 /franka_control/error_recovery/goal franka_msgs/ErrorRecoveryActionGoal "{}"
