#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2 
from sensor_msgs.msg import LaserScan 
import numpy as np
import struct
import sensor_msgs.point_cloud2 as pc2
import tf
def ppl_angles(points):
    #calculate the angle of the first point
    angles=[]
    for pt in points:
        if pt[0]==0:
            angle=3.14/2
        else:
            angle=np.math.atan(pt[1]/pt[0])
        if pt[0]<0 and pt[1]>0:
            angle=angle+ 3.14
        if pt[0]<0 and pt[1]<0:
            angle=angle+ 3.14
        if pt[0]>0 and pt[1]<0:
            angle=angle+ 3.14*2
        angles.append(angle)
    print(' angles ', np.array(angles)*360/6.28)
    return angles

def convert_to_scan(ppl, header):
    global tf_listener
# ppl 1024 points in 16 channel, so scan should have floor(1024/16) 
    camera_translation_base=[0,0,0]
    base_frame='SimpleFlight'
    camera_frame='SimpleFlight/odom_local_ned'
    #camera_frame='SimpleFlight/LidarCustom'
    if tf_listener.frameExists(base_frame) and tf_listener.frameExists(camera_frame):
            # Get transformation
            try:
                time = tf_listener.getLatestCommonTime(base_frame, camera_frame)
                camera_translation_base, camera_orientation_base = tf_listener.lookupTransform(base_frame, camera_frame, time)
            except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                print('tf lookuptransform fail ', base_frame, camera_frame)
                return

            print(camera_frame, ' pose ', camera_translation_base)

    scanmsg = LaserScan()
#    scanmsg.header.frame_id='SimpleFlight'
#    scanmsg.header.frame_id='odom_frame'
    scanmsg.header.frame_id='SimpleFlight/odom_local_ned'
    #scanmsg.header.frame_id='SimpleFlight/LidarCustom'
    #scanmsg.header.frame_id='front_left_custom_optical/static'
    #scanmsg.header.frame_id='front_left_custom_optical'
    scanmsg.header.stamp= header.stamp 
    #scanmsg.header.stamp= rospy.get_rostime()
    #scanmsg.header.seq=header.seq
    scanmsg.range_min=0
    scanmsg.range_max=50
    scansize = int(len(ppl)/16)

#   the ppl are in SimpleFlight frame, need to convert it to the lidar frame for proper display in rviz. after conversion, the scan data point only means the xy location of lidar point. 
#   for now it is sqrt(x^2+y^2) of the lidar reflection. could be ground, could be from an obstacle. there is no telling. will change it later so if its ground point then change it to max range. note that the mapping process will ignore the max range lidar points when building map.

    ppl_lidarframe = np.array(list(ppl))-camera_translation_base
    #ppl_lidarframe = np.array(list(ppl))
    outrim = ppl_lidarframe[0:64]

    #calculate the angle of the first point
    if ppl_lidarframe[0,0]==0:
        angle_start=3.14/2
    else:
        angle_start=np.math.atan(ppl_lidarframe[0,1]/ppl_lidarframe[0,0])
    if ppl_lidarframe[0,0]<0 and ppl_lidarframe[0,1]>0:
        angle_start=angle_start+ 3.14
    if ppl_lidarframe[0,0]<0 and ppl_lidarframe[0,1]<0:
        angle_start=angle_start+ 3.14
    if ppl_lidarframe[0,0]>0 and ppl_lidarframe[0,1]<0:
        angle_start=angle_start+ 3.14*2
    print(' start angle ', angle_start*360/6.28, ppl_lidarframe[0,0], ppl_lidarframe[0,1], ppl_lidarframe[1,0], ppl_lidarframe[1,1])
    #ppl_angles(ppl_lidarframe[0:64]) #debugging, print the angle of the 64 points

    #scanmsg.angle_min = -3.14 
    scanmsg.angle_min = angle_start
    scanmsg.angle_max = angle_start + 3.14*2
    #scanmsg.angle_max =  3.14
    scanmsg.angle_increment = (scanmsg.angle_max - scanmsg.angle_min) / scansize

    #outrimlist=[]
    #for pt in outrim:
    #    outrimlist.append([pt.x, pt.y])

    print('pt1 ', type(outrim[0]), outrim.shape)
    newrim = ppl_lidarframe[0:64]
    for i in range(1,1):
        newrim = ppl_lidarframe[i*64:(i+1)*64] 
        for j in range(len(newrim)):
            if newrim[j][2] < -0.3:
                outrim[j]=newrim[j]
                #print('replace')
    max_x = ppl_max_x(ppl[0:64])
    i=10 
    scanmsg.ranges=np.linalg.norm(ppl_lidarframe[i*64:(i+1)*64,0:2], axis=1)
    #print(scanmsg.ranges)
    #scanmsg.ranges=[max_x]*64
    #scanmsg.ranges=np.linalg.norm(newrim, axis=1)
    #scanmsg.ranges=np.linalg.norm(outrim, axis=1)
    return scanmsg


##################### utility, print max x of the points
def ppl_max_x(points):
    xl=[]
    for pp in points:
        xl.append(pp.x)
    print('ppl max x ', max(xl))
    return max(xl)

###################### return a dummy lidar scan
def laser_dummy(val):
    scanmsg = LaserScan()
    scanmsg.header.frame_id='SimpleFlight'
    scanmsg.header.seq=0
    scanmsg.angle_min=0
    scanmsg.angle_max=3.14*2 #360 deg
    scanmsg.range_min=0
    scanmsg.range_max=50
    scansize = 64 
    scanmsg.angle_increment = (scanmsg.angle_max - scanmsg.angle_min) / scansize
    scanmsg.ranges=[val]*64
    return scanmsg

################ callback when get point cloud 2 from airsim node ##################
def callback(data):
    global laserscan, pplheader,ppl, max_x, lidarnexttime, stoppublishing
    rospy.loginfo(rospy.get_caller_id() + " frameid %s width %d data in bytes %d", data.header.frame_id, data.width, len(data.data))
    points_list = []
    ppl = pc2.read_points_list(data, skip_nans=True)
    pplheader=data.header
    max_x =ppl_max_x(ppl[0:64])
    print('pc2 ', len(ppl), ppl[0])
    return

##### async lidar publishing, do the conversion and publish
def publish_lidar(event=None):
    global laserpub, laserscan, ppl, pplheader, lidarnexttime
    print('lidar pub ...')
    curr_time = rospy.get_rostime()
    if curr_time.secs > lidarnexttime.secs+1:
        print('no lidar data, stop publishing')
        return
    #laserscan= laser_dummy(max_x) 
    if not ppl==None:
        laserscan = convert_to_scan(ppl, pplheader)
    #print('publish_lidar ', laserscan)
    if laserscan==None:
        print('on lidar data...')
        return
    else:
        print('publising lidar ...')
        laserpub.publish(laserscan)

def listener():
    global laserpub, hpub, tf_listener, laserscan, ppl, max_x, lidarnexttime,stoppublishing
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('lidarlistener', anonymous=True)
    ppl=None
    max_x=0
    lidarnexttime=rospy.get_rostime()
    tf_listener = tf.TransformListener()

    rospy.Subscriber("/airsim_node/SimpleFlight/lidar/LidarCustom", PointCloud2, callback)
    laserscan=None
    stoppublishing=True
    laserpub = rospy.Publisher('/scan', LaserScan, queue_size=1)
    hpub = rospy.Publisher('/hi', String, queue_size=10)
#    rospy.Timer(rospy.Duration(1.0/10.0), publish_lidar)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
