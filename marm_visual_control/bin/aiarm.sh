#!/bin/bash

 
sudo chmod 666 /dev/ttyUSB*
source /opt/ros/kinetic/setup.bash
source ~/catkin_ws/devel/setup.bash

cd /home/zonesion/catkin_ws/src/marm_visual_control/script

roslaunch marm_visual_control aiarm-controller.launch &
sleep 26
python main.py
sleep 99999
