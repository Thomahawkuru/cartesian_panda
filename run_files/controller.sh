#!/usr/bin/env bash

cd
cd tele_panda_ws
source devel/setup.bash

xterm -hold -e "roslaunch cartesian_panda cartesian_impedance_controller.launch"

