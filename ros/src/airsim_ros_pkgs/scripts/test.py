#!/usr/bin/env python  
import rospy

import geometry_msgs.msg
import turtlesim.msg
from nav_msgs.msg import Odometry
import numpy as np
import sys
import time
import argparse

def handle_turtle_pose(event=None):
#def handle_turtle_pose(msg, turtlename):
    print('tf xxx to base_footprint')
    t = geometry_msgs.msg.TransformStamped()

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--weights', nargs='+', type=str,
                        default='yolov5s.pt', help='model.pt path(s)')
    for indx in range(len(sys.argv)):
        if sys.argv[indx].find('--')==0:
            break
    print(sys.argv[indx:])
    opt = parser.parse_args(sys.argv[indx:])
    print(opt)
    rospy.init_node('tf2_turtle_broadcaster')
    turtlename = 'tftest'
    rx = np.random.uniform()
    print(rx)
    #rospy.Timer(rospy.Duration(1.0/10.0), handle_turtle_pose)
    rospy.Subscriber('/odom' ,
                     Odometry,
                     handle_turtle_pose,
                     turtlename)
    rospy.spin()
