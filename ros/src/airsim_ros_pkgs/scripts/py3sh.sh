#!/bin/bash
echo 'py3sh ', $1, $2
source /home/student/bin/py36torch.sh
cd /home/student/Documents/AirSim/ros/src/airsim_ros_pkgs/scripts
rosrun airsim_ros_pkgs state.py --ip $1

