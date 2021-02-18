#!/bin/bash
echo 'py3land ', $1, $2
source /home/student/bin/py36torch.sh
cd /home/student/Documents/AirSim/ros/src/airsim_ros_pkgs/scripts
rosrun airsim_ros_pkgs land.py --ip $1
