#!/bin/bash

cd /home/zonesion/catkin_ws
catkin_make
cd /home/zonesion/catkin_ws/src/marm_controller/src
chmod 777 *
cd /home/zonesion/catkin_ws/src/marm_visual_control/bin
chmod 777 *

