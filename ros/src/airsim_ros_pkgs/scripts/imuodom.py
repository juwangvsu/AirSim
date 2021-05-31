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
import tf
import tf.transformations as transformations

def write_imu(imudata):
    global imufile, imufilewrite
    #imufile.write(str(points)+'\n')
    tstamp = imudata.header.stamp.secs + imudata.header.stamp.nsecs/1000000000.0
    linear_acc_body=[imudata.linear_acceleration.x, imudata.linear_acceleration.y, imudata.linear_acceleration.z, 1]
    imupart = [tstamp, imudata.linear_acceleration.x, imudata.linear_acceleration.y, imudata.linear_acceleration.z, imudata.angular_velocity.x, imudata.angular_velocity.y, imudata.angular_velocity.z]

    body_orientation_quat =[imudata.orientation.x, imudata.orientation.y,imudata.orientation.z,imudata.orientation.w]
    m_b2w = transformations.quaternion_matrix(body_orientation_quat)
    # pure rotation matrix, 4x4. nor translation here since we are transforming
    # a vector in the body frame.
    #m_b2w = transformations.quaternion_matrix([0, 0, np.sin(np.pi/8), np.cos(np.pi/8)])# this rotation matrix is 4x4 and tranform from body to world frame, 45 deg
    #Quaternions ix+jy+kz+w are represented as [x, y, z, w].
    gravity_acc_world = [0,0, -9.8, 0]
    #print(m_b2w)
    linear_acc_world = np.dot(m_b2w,linear_acc_body) - gravity_acc_world
    print(list(linear_acc_world))
    imufilewrite.writerow(imupart + list(linear_acc_world))

#def handle_turtle_pose(event=None):
def handle_turtle_imu(msg, turtlename):
    global imufile
#    print('tf xxx to base_footprint', msg)
    write_imu(msg)
#    t = geometry_msgs.msg.TransformStamped()


def write_odom(odomdata):
    global odomfile, odomfilewrite
    #imufile.write(str(points)+'\n')
    tstamp = odomdata.header.stamp.secs + odomdata.header.stamp.nsecs/1000000000.0
    xyz_pose=odomdata.pose.pose.position
    xyz_vel=odomdata.twist.twist.linear
    quat=odomdata.pose.pose.orientation
    odomfilewrite.writerow([tstamp, xyz_pose.x, xyz_pose.y, xyz_pose.z, xyz_vel.x, xyz_vel.y, xyz_vel.z, quat.x, quat.y, quat.z, quat.w])

#def handle_turtle_pose(event=None):
def handle_turtle_odom(msg, turtlename):
    global odomfile
    write_odom(msg)

if __name__ == '__main__':
    global imufile, imufilewrite
    global odomfile, odomfilewrite
    rospy.init_node('tf2_turtle_broadcaster')
    turtlename = 'tftest'
    rx = np.random.uniform()
    print(rx)
    print('save imu data to imu0.txt, linear acc xyz, angular vel xyz')
    imufile = open('imu0.txt', 'w+')
    imufilewrite = csv.writer(imufile)
    imufilewrite.writerow(['t', 'xa', 'ya', 'za', 'avx', 'avy', 'avz','xa_w','ya_w','za_w','1 xa_w in world frame with gravity removed, xa in body frame'])
    #rospy.Timer(rospy.Duration(1.0/10.0), handle_turtle_pose)
    rospy.Subscriber('/airsim_node/SimpleFlight/imu/Imu' ,
                     Imu,
                     handle_turtle_imu,
                     turtlename)
    odomfile = open('odom0.txt', 'w+')
    odomfilewrite = csv.writer(odomfile)
    odomfilewrite.writerow(['t', 'x', 'y', 'z', 'vx', 'vy', 'vz', 'qx', 'qy', 'qz', 'qw'])
    #rospy.Timer(rospy.Duration(1.0/10.0), handle_turtle_pose)
    rospy.Subscriber('/airsim_node/SimpleFlight/odom_local_ned' ,
                     Odometry,
                     handle_turtle_odom,
                     turtlename)

    rospy.spin()
