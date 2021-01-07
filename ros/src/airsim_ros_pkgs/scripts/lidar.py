#!/usr/bin/env python
import sys
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2 
from sensor_msgs.msg import LaserScan 
import numpy as np
import struct
import sensor_msgs.point_cloud2 as pc2
import tf
from readscan import readlidardummy
import argparse
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

def convert_to_scan(ppl, header, shiftscan=False):
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
    print(' start angle ', angle_start*360/6.28, 'cam_trans_base ', camera_translation_base )
    #ppl_angles(ppl_lidarframe[0:64]) #debugging, print the angle of the 64 points

    #scanmsg.angle_min = -3.14 
    scanmsg.angle_min = angle_start
    scanmsg.angle_max = angle_start + 3.14*2
    #scanmsg.angle_max =  3.14
    scanmsg.angle_increment = (scanmsg.angle_max - scanmsg.angle_min) / scansize

    #outrimlist=[]
    #for pt in outrim:
    #    outrimlist.append([pt.x, pt.y])

    #print('pt1 ', type(outrim[0]), outrim.shape) #<type 'numpy.ndarray'>, (64, 3))
    newrim = ppl_lidarframe[0:64]
    for i in range(1,1):
        newrim = ppl_lidarframe[i*64:(i+1)*64] 
        for j in range(len(newrim)):
            if newrim[j][2] < -0.3:
                outrim[j]=newrim[j]
                #print('replace')
    max_x = ppl_max_x(ppl[0:64])
    # lidar in turtlebot is dense, 0.0175019223243 deg angle inc, total 360 points
    # here we have angle_increment: 0.0981250032783, total 64 points
    # duplicate by 6 times, and scale the range by 0.3 for gmapping to work.
    i=0 
    scanmsg.angle_increment = scanmsg.angle_increment/6
    extended_ppl = 0.3*np.tile(ppl_lidarframe[i*64:(i+1)*64,0:2], 6).reshape(64*6,2)
    scanmsg.ranges= np.linalg.norm(extended_ppl, axis=1)
    #print(scanmsg.ranges)
    #scanmsg.ranges=[max_x]*64
    #scanmsg.ranges=np.linalg.norm(newrim, axis=1)
    #scanmsg.ranges=np.linalg.norm(outrim, axis=1)
    print('shift rad ', scanmsg.angle_min, scanmsg.angle_max)
    if shiftscan == 1:
       print('shift rad ', scanmsg.angle_min, scanmsg.angle_max)
       shiftptnum = int(scanmsg.angle_min/scanmsg.angle_increment)
       scanmsg.ranges=np.roll(scanmsg.ranges, shiftptnum)
       scanmsg.angle_min=0
       scanmsg.angle_max=6.28
    return scanmsg


##################### utility, print max x of the points
def ppl_max_x(points):
    xl=[]
    for pp in points:
        xl.append(pp.x)
    #print('ppl max x ', max(xl))
    return max(xl)

###################### return a dummy lidar scan
# shift_rad: shift the points in ranges and angle_min etc by a degree in rad, assume positive
def laser_dummyfile(shift_rad=0):
    lidardummymsg = readlidardummy()
    scanmsg = LaserScan()
    scanmsg.header.frame_id=lidardummymsg['header']['frame_id']
    scanmsg.header.seq=0
    scanmsg.angle_min=0 + shift_rad
    scanmsg.angle_max=3.14*2 +shift_rad #360 deg
    scanmsg.range_min=0
    scanmsg.range_max=50
    scanmsg.angle_increment = lidardummymsg['angle_increment'] 
    shiftptnum = int(shift_rad/scanmsg.angle_increment)

    scanmsg.ranges=lidardummymsg['ranges']
    scanmsg.ranges=np.roll(scanmsg.ranges, -shiftptnum)
    return scanmsg

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
    #rospy.loginfo(rospy.get_caller_id() + " frameid %s width %d data in bytes %d", data.header.frame_id, data.width, len(data.data))

#important! check if data point is 1024. if not, the data order is not normal and mess up the conversion start angle, which result in a rotation offset in scan.
    if not data.width == 1024:
        print(' return on data len ', data.width)
        return
    curr_time = rospy.get_rostime()

    lidarnexttime=curr_time

    #print('lidar msg time offset ', data.header.stamp.secs-curr_time.secs, data.header.stamp.nsecs-curr_time.nsecs)

# proceed to process pc2, save the current pc2 in global variable for async process. otherwise the conversion time too much and cause strange buffer delay.
    points_list = []
    #for point in pc2.read_points(data, skip_nans=True):
    #   points_list.append([point[0], point[1], point[2]])
    #print(len(points_list), points_list[0])
    ppl = pc2.read_points_list(data, skip_nans=True)
    pplheader=data.header
    max_x =ppl_max_x(ppl[0:64])
    #print('pc2 ', len(ppl), ppl[0])
    #pointslist=list(struct.unpack('2f', my_str_as_bytes))
    #points = np.array(pointslist, dtype=np.dtype('f4'))

    # convert_to_scan(ppl, data.header) # this step cpu hungery, cause delay in published laser, so should do this async at the publish laser step.
 
scanmsg_count=0
##### async lidar publishing, do the conversion and publish
def publish_lidar(event=None):
    global laserpub, laserscan, ppl, pplheader, lidarnexttime, scanmsg_count, dummytest, shiftscan
    curr_time = rospy.get_rostime()
    #print('lidar pub ...',  curr_time.nsecs, lidarnexttime.nsecs )
    if curr_time.secs > lidarnexttime.secs+1 and not dummytest:
    #if curr_time.secs > lidarnexttime.secs or curr_time.nsecs>lidarnexttime.nsecs+800000000:
        print('no lidar data, stop publishing')
        return
    if dummytest:
        #laserscan= laser_dummy(max_x) 
        laserscan= laser_dummyfile(scanmsg_count*0.02) 
    if not ppl==None and not dummytest:
        laserscan = convert_to_scan(ppl, pplheader, shiftscan=shiftscan)
    #print('publish_lidar ', laserscan)
    if laserscan==None:
        print('on lidar data...')
        return
    else:
        #print('publising lidar ...', scanmsg_count)
        #laserscan.range_max = laserscan.range_max + scanmsg_count*0.1
        scanmsg_count = scanmsg_count +1
        laserpub.publish(laserscan)

def listener():
    global laserpub, hpub, tf_listener, laserscan, ppl, max_x, lidarnexttime,stoppublishing, dummytest
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
    #hpub = rospy.Publisher('/hi', String, queue_size=10)
    rospy.Timer(rospy.Duration(1.0/10.0), publish_lidar)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    global dummytest, shiftscan
    dummytest=True
    dummytest=False
    args = sys.argv
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description="lidar arg parser")
    parser.add_argument("--shiftscan", action="store", type=int, help="shift lidar scan", default=0)
    args = parser.parse_args(rospy.myargv()[1:])
    print('shiftscan ', args)
    shiftscan=args.shiftscan
    listener()
