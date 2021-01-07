#!/usr/bin/env python
import rospy
import rosbag

import os
import sys
import textwrap

import yaml
lidarmsg=None
################# read the lidar msg from yaml file and return ##############
def readlidardummy():
  global lidarmsg
  if lidarmsg==None:
      lidarmsg= doreadlidar()
  return lidarmsg

def doreadlidar():
  global lidarmsg
  print('lidardummy do read')
  with open(r'/media/student/data5/AirSim/ros/src/airsim_ros_pkgs/scripts/lidar_dummy.txt') as file:
    # The FullLoader parameter handles the conversion from YAML
    # scalar values to Python the dictionary format
    lidarmsg = yaml.load(file)

    #print(fruits_list)
    #print(lidarmsg['range_max']+20)
    #print(lidarmsg['header']['stamp']['secs'])
    ranges=lidarmsg['ranges']
    #print(len(ranges), ranges)
    return lidarmsg

if __name__ == '__main__':
    readlidardummy()

