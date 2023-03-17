#!/usr/bin/env bash
cd
cd tele_panda_ws
source devel/setup.bash

xterm -e "roslaunch cartesian_panda reset.launch"
