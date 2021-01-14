#!/usr/bin/env python
#subscribe to /airsim_node/SimpleFlight/lidar/LidarCustom
#republish point cloud2 to examine one of the 16 channel of the data
# published topic /pc2
# each channel has <64 data points
# to change the data channel
#rosparam set /segid 5
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2 
from sensor_msgs.msg import LaserScan 
import numpy as np
import struct
import sensor_msgs.point_cloud2 as pc2
import tf
import sys
import argparse
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
import std_msgs.msg
def ppl_angles(points):
    #calculate the angle of the first point, 0 - 2pi
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
    #print(' angles ', np.array(angles)*360/6.28)
    return angles

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
    scanmsg.angle_min=0.5
    scanmsg.angle_max=1.14 #360 deg
    scanmsg.range_min=0
    scanmsg.range_max=50
    scansize = 64 
    scanmsg.angle_increment = 0.2# (scanmsg.angle_max - scanmsg.angle_min) / 35
    scanmsg.ranges=[val]*14
    return scanmsg

#break data.data into 16 segments according to the datapoint angles
#p_angle: angle in rad
#datalist: list of data in bytes, lenth is 12*len(p_angle)
def ppl_16seg(p_angle, datalist):
    segs=[]
    segs_index=[]
    startind=0
    endind=0
    #print('p_angle ', p_angle)
    for i in range(40):
        if startind+1==len(p_angle):
            break
        if p_angle[startind]> p_angle[startind+1]:
            startind = startind +1
            continue
        for j in range(startind+1, len(p_angle)-1):
            if p_angle[j] < p_angle[startind] and p_angle[j+1]>= p_angle[startind]:
               break
        #print(i, 'th loop start, end ', startind, j)
        seg_i = datalist[startind*12: j*12]
        seg_index = [startind, j]
        if len(seg_i) >5*12:
                segs.append(seg_i)
                segs_index.append(seg_index)
        if len(seg_i) >3000:
               print( 'len(seg_i)', len(seg_i), 'p_angle ', p_angle)
               eexit(1)
        startind = j+1

    #print('16 segment: ', segs)
    return segs, segs_index



################ callback when get point cloud 2 from airsim node ##################
def callback(data):
    global laserscan, pplheader,ppl, p_angles, p_segs, p_segindex,max_x, lidarnexttime, stoppublishing, pc2pub, pc2data
#    rospy.loginfo(rospy.get_caller_id() + " frameid %s width %d data in bytes %d", data.header.frame_id, data.width, len(data.data))
    points_list = []
    ppl = pc2.read_points_list(data, skip_nans=True)
    p_angles = ppl_angles(ppl)
    p_segs, p_segindex = ppl_16seg(p_angles, data.data)

    pplheader=data.header
    #max_x =ppl_max_x(ppl[0:64])
    #print('pc2 ', len(ppl), ppl[0])

    segid = rospy.get_param("/segid")
    data.width=len(p_segs[segid])/12
    data.row_step=data.width*12
    data.data=p_segs[segid]
    pc2data=data
    #data.data=data.data[0:data.row_step]
    return

def convert_to_scan(pplseg_sel, pangles_sel, header, shiftscan=False):
    scanmsg = LaserScan()
    scanmsg.header=header
    scanmsg.range_min=0
    scanmsg.range_max=50
    scansize = len(pplseg_sel)
    angle_start=pangles_sel[0]
    print('angle start ', angle_start)
    scanmsg.angle_min = angle_start
    scanmsg.angle_max = angle_start + 3.14*2
    scanmsg.angle_increment = (scanmsg.angle_max - scanmsg.angle_min) / 64
    extended_ppl = np.array(pplseg_sel)[:,0:2]
    scanmsg.ranges= np.linalg.norm(extended_ppl, axis=1)

    pangles_sel = ppl_angles(pplseg_sel)
    print('before filled points xy  ', extended_ppl)
    print('before filled angle  ', np.array(pangles_sel))
    print('before filled ranges  ', scanmsg.ranges)
    scanmsg.ranges = fillmissing2(pangles_sel, scanmsg.ranges, scanmsg.range_max)
    #scanmsg.ranges = fillmissing(pangles_sel, scanmsg.ranges, scanmsg.range_max)
    scanmsg.angle_increment = (scanmsg.angle_max - scanmsg.angle_min) / len(scanmsg.ranges)
    scanmsg.angle_min=0
    scanmsg.angle_max=6.28
    print('filled ranges  ', scanmsg.ranges)


    #shift so the first point is always at 0 deg
    print('shift rad ', scanmsg.angle_min, scanmsg.angle_max)
    if shiftscan == 1:
       shiftptnum = int(scanmsg.angle_min/scanmsg.angle_increment)
       print('shift rad ', scanmsg.angle_min, scanmsg.angle_max, '#pts ', shiftptnum)
       scanmsg.ranges=np.roll(scanmsg.ranges, shiftptnum)
       scanmsg.angle_min=0
       scanmsg.angle_max=6.28
       print('aftersift ranges  ', scanmsg.ranges)
    slopesample=0
    for ind in range(90,100):
        if np.abs(scanmsg.ranges[ind]-scanmsg.ranges[ind+1]) < 30:
            slopesample = slopesample+scanmsg.ranges[ind]-scanmsg.ranges[ind+1]
    if slopesample > 20 or slopesample<-20:
        print('ranges 240-290: ', scanmsg.ranges[240:260])

    return scanmsg, slopesample

############################################################3
# assume all angles 0 - 6.28
# no shift needed`
def fillmissing2(angles_sel, ranges, range_max):
   ranges_list = list(ranges)
   new_ranges=[50]*384
   new_angle_inc=6.28/384
   for i in range(len(ranges)-1):
       i_loc = int(angles_sel[i] / new_angle_inc)
       new_ranges[i_loc]=ranges[i]
#   return np.array(new_ranges)

#linear interpretation btw two consecutive points:
   for i in range(len(ranges)-2):
#       print('ang diff ', np.abs(angles_sel[i]-angles_sel[i+1]), 2*new_angle_inc)
       if np.abs(angles_sel[i]-angles_sel[i+1])<10*new_angle_inc:
            i_loc1 = int(angles_sel[i] / new_angle_inc)+1
            i_loc2 = int(angles_sel[i+1] / new_angle_inc)
            for j in range(i_loc1, i_loc2):
                new_ranges[j]=ranges[i] + (ranges[i+1]-ranges[i])*(j-i_loc1)/(i_loc2-i_loc1)
   return np.array(new_ranges)

########################################################################################
# each channel suppose has 64 points, angle_incr = 0.098125, we pad the missing data
# to make it 64 points. missing point has a max range=50 
# assuming angle increase order, except at cross over from 2pi to 0
def fillmissing(angles_sel, ranges, range_max):
   ranges_list = list(ranges)
   new_ranges=[]
   for i in range(len(ranges)-1):
       new_ranges.append(ranges[i])
       if angles_sel[i]<angles_sel[i+1]:
            pt_gap = int((np.abs(angles_sel[i+1]-angles_sel[i])+0.05)/0.098125)-1
       else:
           #cross over 2pi
            if np.abs(angles_sel[i+1]-angles_sel[i])<1:
                pt_gap=0 # this is abnormal case where angle decrease at non cross over, igonre
            else:
                pt_gap = np.abs(int((6.28-angles_sel[i]+angles_sel[i+1]+0.05)/0.098125))-1

       for j in range(pt_gap):
           new_ranges.append(range_max)
   new_ranges.append(ranges[len(ranges)-1])
   endpadding = 64- len(new_ranges)
   for k in range(endpadding):
       new_ranges.append(range_max)
   return np.array(new_ranges)

def publish_pc2(event=None):
    global pc2data, ppl, p_angles,p_segs, p_segindex, laserpub, shiftscan
    segid = rospy.get_param("/segid")
    if pc2data==None:
        pass
    else:
        ppl_seg=ppl[p_segindex[segid][0]:p_segindex[segid][1]]
        pangle_seg=p_angles[p_segindex[segid][0]:p_segindex[segid][1]]
        #print('segid ', segid, p_segindex[segid], ppl_seg)
        pc2pub.publish(pc2data)
        my_pc = PointCloud()
        header = std_msgs.msg.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = pc2data.header.frame_id
        my_pc.header = header
        for pt in ppl_seg:
            my_pc.points.append(pt)
        print "happily publishing pointcloud"
        pc_pub.publish(my_pc)

        #/scan
        #laserscan = laser_dummy(3)
        laserscan,slopesample = convert_to_scan(ppl_seg, pangle_seg, my_pc.header, shiftscan)
        laserpub.publish(laserscan)
        if slopesample > 20 or slopesample<-20:
            print(' slope exceed 20 abnormal')
            #rospy.signal_shutdown('slope abnormal')


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
    global laserpub, hpub, tf_listener, laserscan, ppl, max_x, lidarnexttime,stoppublishing,pc2pub, pc2data, pc_pub
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('lidarlistener', anonymous=True)
    ppl=None
    max_x=0
    pc2data=None
    rospy.set_param('/segid', 11)
    lidarnexttime=rospy.get_rostime()
    #tf_listener = tf.TransformListener()

    rospy.Subscriber("/airsim_node/SimpleFlight/lidar/LidarCustom", PointCloud2, callback)
    laserscan=None
    stoppublishing=True
    laserpub = rospy.Publisher('/scan', LaserScan, queue_size=1)
    pc2pub = rospy.Publisher('/pc_2', PointCloud2, queue_size=1)
    rospy.Timer(rospy.Duration(1.0/10.0), publish_pc2)
    pc_pub = rospy.Publisher("/pc", PointCloud)
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
