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
def write_imu(odomdata):
    global imufile, imufilewrite
    #imufile.write(str(points)+'\n')
    tstamp = odomdata.header.stamp.secs + odomdata.header.stamp.nsecs/1000000000.0
    xyz_pose=odomdata.pose.pose.position
    xyz_vel=odomdata.twist.twist.linear
    quat=odomdata.pose.pose.orientation
    imufilewrite.writerow([tstamp, xyz_pose.x, xyz_pose.y, xyz_pose.z, xyz_vel.x, xyz_vel.y, xyz_vel.z, quat.x, quat.y, quat.z, quat.w])

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
    print('save odom data to imu0.txt, linear acc xyz, angular vel xyz')
    imufile = open('odom0.txt', 'w+')
    imufilewrite = csv.writer(imufile)
    imufilewrite.writerow(['t', 'x', 'y', 'z', 'vx', 'vy', 'vz', 'qx', 'qy', 'qz', 'qw'])
    #rospy.Timer(rospy.Duration(1.0/10.0), handle_turtle_pose)
    rospy.Subscriber('/airsim_node/SimpleFlight/odom_local_ned' ,
                     Odometry,
                     handle_turtle_pose,
                     turtlename)
    rospy.spin()
