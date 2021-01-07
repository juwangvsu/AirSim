#!/usr/bin/env python  
import rospy

# Because of transformations
import tf_conversions

import tf2_ros
import geometry_msgs.msg
import turtlesim.msg
from nav_msgs.msg import Odometry
import numpy as np
def handle_turtle_pose(event=None):
#def handle_turtle_pose(msg, turtlename):
    print('tf xxx to base_footprint')
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "xxx"
    t.child_frame_id = "base_footprint" 
    rx = np.random.uniform()
    t.transform.translation.x = 0 + rx
    t.transform.translation.y = 1
    t.transform.translation.z = 0.0
    q = tf_conversions.transformations.quaternion_from_euler(0, 0, 3.14/2)
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]
    br.sendTransform(t)

if __name__ == '__main__':
    rospy.init_node('tf2_turtle_broadcaster')
    turtlename = 'tftest' 
    rx = np.random.uniform()
    print(rx)
    rospy.Timer(rospy.Duration(1.0/10.0), handle_turtle_pose)
    #rospy.Subscriber('/odom' ,
    #                 Odometry,
    #                 handle_turtle_pose,
    #                 turtlename)
    rospy.spin()
