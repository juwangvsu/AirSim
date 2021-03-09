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
    global odom_adjust
    print('ppl_angles odom_adjust:', odom_adjust)
    camera_translation_base=[0,0,0]
    if odom_adjust: # and False:
        camera_translation_base=odom_adjust_pose()
        print('ppl_angles odom_adjust:', camera_translation_base)
        points = np.array(list(points))-camera_translation_base
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

# segment 16 channel data using angle info. p_angle size is about 64*16
# datalist is raw data in byte list format, size about 64*16*12, each point xyz 4 bytes
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
            # normal case the end point angle cond:
            if p_angle[j] < p_angle[startind] and p_angle[j+1]>= p_angle[startind]:
               break
           # for cases the start point is close to 0 and the supposely end point angle close to 2pi
            if p_angle[j] >4 and p_angle[startind]<1 and p_angle[j+1]>= p_angle[startind]:
               break
        #print(i, 'th loop start, end ', startind, j)
        seg_i = datalist[startind*12: j*12]
        seg_index = [startind, j]
        if len(seg_i) >5*12:
                segs.append(seg_i)
                segs_index.append(seg_index)
        if len(seg_i) >3000:
               print( 'len(seg_i)', len(seg_i),'startind ', startind,  'p_angle ', np.array(p_angle).reshape(-1,4))
               eexit(1)
        startind = j+1

    #print('16 segment: ', segs)
    return segs, segs_index

########################################################################3
# return the pose of odom for point cloud adjustment if needed
def odom_adjust_pose():
    global odom_adjust
    camera_translation_base=[0,0,0]
    base_frame='SimpleFlight'
    camera_frame='SimpleFlight/odom_local_ned'
    print('odom_adjust_pose ', odom_adjust, tf_listener.frameExists(base_frame),tf_listener.frameExists(camera_frame))
    if odom_adjust and tf_listener.frameExists(camera_frame):
    #if odom_adjust and tf_listener.frameExists(base_frame) and tf_listener.frameExists(camera_frame):
            print('odom_adjust_pose try')
            # Get transformation
            try:
                time = tf_listener.getLatestCommonTime(base_frame, camera_frame)
                camera_translation_base, camera_orientation_base = tf_listener.lookupTransform(base_frame, camera_frame, time)
            except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                print('tf lookuptransform fail ', base_frame, camera_frame)
                return

            print('odom_adjust ', camera_frame, ' pose ', camera_translation_base)
    return camera_translation_base

##############################################################################3
def convert_to_scan(ppl, p_angles, p_segindex, header, shiftscan=False):
    global tf_listener, odom_adjust
# ppl 1024 points in 16 channel, so scan should have floor(1024/16) 
    base_frame='SimpleFlight'
    camera_frame='SimpleFlight/odom_local_ned'

    scanmsg = LaserScan()
    #scanmsg.header.frame_id='SimpleFlight/odom_local_ned'
    scanmsg.header.frame_id='SimpleFlight/odom_local_ned'
    scanmsg.header.stamp= header.stamp 
    #scanmsg.header.stamp= rospy.get_rostime()
    #scanmsg.header.seq=header.seq
    scanmsg.range_min=0
    scanmsg.range_max=50
    segid = rospy.get_param("/segid")
    if segid<0:
        # for segid<0, use a 64 points slice directly
        i=np.abs(segid)-1
        pplseg_sel = ppl[i*64:(i+1)*64]
    else:
        pplseg_sel = ppl[p_segindex[segid][0]:p_segindex[segid][1]]
    angles_sel = ppl_angles(pplseg_sel) # this instead of using p_angles[] directly to deal with misterious data inconsistancy btw ppl and p_angles
    scansize = len(pplseg_sel)

#   the ppl are in SimpleFlight frame, need to convert it to the lidar frame for proper display in rviz. after conversion, the scan data point only means the xy location of lidar point. 
#   for now it is sqrt(x^2+y^2) of the lidar reflection. could be ground, could be from an obstacle. there is no telling. will change it later so if its ground point then change it to max range. note that the mapping process will ignore the max range lidar points when building map.
    camera_translation_base=[0,0,0]
    if odom_adjust:
        camera_translation_base=odom_adjust_pose()
    ppl_lidarframe = np.array(list(pplseg_sel))-camera_translation_base

    #print('ppl_lidarframe ',ppl_lidarframe)
    #print('pplseg_sel ',p_segindex[segid], 'angles', angles_sel)
    #ppl_lidarframe = np.array(list(ppl))

    angle_start=angles_sel[0]
    print(' start angle ', angle_start*360/6.28, 'cam_trans_base ', camera_translation_base )
    #ppl_angles(ppl_lidarframe[0:64]) #debugging, print the angle of the 64 points

    scanmsg.angle_min = angle_start # normally min 0, max 2pi
    scanmsg.angle_max = angle_start + 3.14*2

    # lidar in turtlebot is dense, 0.0175019223243 deg angle inc, total 360 points
    # here we have angle_increment: 0.0981250032783, total 64 points
    # duplicate by 6 times, and scale the range by 0.3 for gmapping to work. this is
    # done inside fillmissing2()
    # each channel suppose has 64 points, angle_incr = 0.098125, we pad the missing data
    # to make it 64 points. missing point has a max range=50 
    extended_ppl = ppl_lidarframe[:,0:2]
    scanmsg.ranges= np.linalg.norm(extended_ppl, axis=1)
    #print('ranges  ', scanmsg.ranges)
    scanmsg.ranges = fillmissing2(angles_sel, scanmsg.ranges, scanmsg.range_max)
    #scanmsg.ranges = fillmissing(pangles_sel, scanmsg.ranges, scanmsg.range_max)
    scanmsg.angle_increment = (scanmsg.angle_max - scanmsg.angle_min) / len(scanmsg.ranges)
    scanmsg.angle_min=0
    scanmsg.angle_max=6.28

    #shift so the first point is always at 0 deg, shift is no longer needed after fillmissing2()
    #kept here for debugging purpose
    if shiftscan == 1:
       shiftptnum = int(scanmsg.angle_min/scanmsg.angle_increment)
       print('shift rad ', scanmsg.angle_min, scanmsg.angle_max, '#pts ', shiftptnum)
       scanmsg.ranges=np.roll(scanmsg.ranges, shiftptnum)
       scanmsg.angle_min=0
       scanmsg.angle_max=6.28
    return scanmsg

#############################################################3
# assume all angles 0 - 6.28, upsample 6 times to 384 point
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
   return new_ranges

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
    global laserscan, pplheader,ppl, max_x, lidartime, stoppublishing, rawlidardata, tf_listener

    base_frame='world'
    camera_frame='SimpleFlight/odom_local_ned'
    print('tf frames', tf_listener.frameExists(base_frame),tf_listener.frameExists(camera_frame))
    return
    #rospy.loginfo(rospy.get_caller_id() + " frameid %s width %d data in bytes %d", data.header.frame_id, data.width, len(data.data))

#important! check if data point is 1024. if not, the data order is not normal and mess up the conversion start angle, which result in a rotation offset in scan.
#this is resolved now. incomplete data size are segmented according to their starting angles
    if not data.width == 1024:
        #print(' return on data len ', data.width)
        #return
        pass
    curr_time = rospy.get_rostime()

    if curr_time.secs == lidartime.secs and curr_time.nsecs-lidartime.nsecs < 20000000:
        return
        pass
    lidartime=curr_time

    #print('lidar msg time offset ', data.header.stamp.secs-curr_time.secs, data.header.stamp.nsecs-curr_time.nsecs)

# proceed to process pc2, save the current pc2 in global variable for async process. otherwise the conversion time too much and cause strange buffer delay.
    points_list = []
    #for point in pc2.read_points(data, skip_nans=True):
    #   points_list.append([point[0], point[1], point[2]])
    #print(len(points_list), points_list[0])
    rawlidardata=data
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
    global laserpub, laserscan, ppl, pplheader, rawlidardata, lidartime, scanmsg_count, dummytest, shiftscan
    curr_time = rospy.get_rostime()
    print('lidar pub ...',  curr_time.nsecs, lidartime.nsecs )
    if curr_time.secs > lidartime.secs+1 and not dummytest:
    #if curr_time.secs > lidartime.secs or curr_time.nsecs>lidartime.nsecs+800000000:
        print('no lidar data, stop publishing')
        return
    if dummytest:
        print('dummytest publishing')
        #laserscan= laser_dummy(max_x) 
        laserscan= laser_dummyfile(scanmsg_count*0.02) 
    if not ppl==None and not dummytest:
        print('publish lidar data to /scan ')
        p_angles = ppl_angles(ppl)
        p_segs,p_segindex = ppl_16seg(p_angles, rawlidardata.data)
        laserscan = convert_to_scan(ppl, p_angles, p_segindex, pplheader, shiftscan=shiftscan)
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
    global laserpub, hpub, tf_listener, laserscan, ppl,rawlidardata, max_x, lidartime,stoppublishing, dummytest
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('lidarlistener', anonymous=True)
    ppl=None
    rawlidardata=None
    max_x=0
    rospy.set_param('/segid', 11)
    lidartime=rospy.get_rostime()
    base_frame='SimpleFlight'
    camera_frame='SimpleFlight/odom_local_ned'
    tf_listener = tf.TransformListener()

    print('tf frames', tf_listener.frameExists(base_frame),tf_listener.frameExists(camera_frame))
    
    rospy.Subscriber("/airsim_node/SimpleFlight/lidar/LidarCustom", PointCloud2, callback)
    laserscan=None
    stoppublishing=True
    laserpub = rospy.Publisher('/scan', LaserScan, queue_size=1)
    #hpub = rospy.Publisher('/hi', String, queue_size=10)
    rospy.Timer(rospy.Duration(1.0/10.0), publish_lidar)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    global dummytest, shiftscan, odom_adjust
    dummytest=True
    dummytest=False
    args = sys.argv
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description="lidar arg parser")
    parser.add_argument("--shiftscan", action="store", type=int, help="shift lidar scan", default=0)
    parser.add_argument("--odom_adjust", action="store", type=bool, help="adjust scan with odom tf", default=False)
    args = parser.parse_args(rospy.myargv()[1:])
    print('shiftscan ', args)
    shiftscan=args.shiftscan
    odom_adjust= args.odom_adjust
    listener()
