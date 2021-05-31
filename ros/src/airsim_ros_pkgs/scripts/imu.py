#!/usr/bin/env python  
import rospy

import geometry_msgs.msg
import sensor_msgs.msg
from sensor_msgs.msg import Imu
import turtlesim.msg
from nav_msgs.msg import Odometry
import numpy as np
import sys
import time
import argparse
import csv
def write_imu(imudata):
    global imufile, imufilewrite
    #imufile.write(str(points)+'\n')
    tstamp = imudata.header.stamp.secs + imudata.header.stamp.nsecs/1000000000.0
    imufilewrite.writerow([tstamp, imudata.linear_acceleration.x, imudata.linear_acceleration.y, imudata.linear_acceleration.z, imudata.angular_velocity.x, imudata.angular_velocity.y, imudata.angular_velocity.z])

#def handle_turtle_pose(event=None):
def handle_turtle_pose(msg, turtlename):
    global imufile
#    print('tf xxx to base_footprint', msg)
    write_imu(msg)
#    t = geometry_msgs.msg.TransformStamped()

if __name__ == '__main__':
    global imufile, imufilewrite
    rospy.init_node('tf2_turtle_broadcaster')
    turtlename = 'tftest'
    rx = np.random.uniform()
    print(rx)
    print('save imu data to imu0.txt, linear acc xyz, angular vel xyz')
    imufile = open('imu0.txt', 'w+')
    imufilewrite = csv.writer(imufile)
    imufilewrite.writerow(['t', 'xa', 'ya', 'za', 'avx', 'avy', 'avz'])
    #rospy.Timer(rospy.Duration(1.0/10.0), handle_turtle_pose)
    rospy.Subscriber('/airsim_node/SimpleFlight/imu/Imu' ,
                     Imu,
                     handle_turtle_pose,
                     turtlename)
    rospy.spin()
