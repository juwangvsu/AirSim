#!/usr/bin/env python
# Python libs
import sys, time
import random

# numpy and scipy
import numpy as np
from scipy.ndimage import filters

# OpenCV
import cv2

# Ros libraries
import roslib
import rospy

# Ros Messages
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage

sys.path.append("/home/student/Documents/yolov5") # change it to your yolov5 dir

from utils.plots import plot_one_box


class image_feature:

    def __init__(self):
        '''Initialize ros subscriber'''
        # subscribed Topic
        #self.subscriber = rospy.Subscriber("/output/color/compressed",
        self.subscriber = rospy.Subscriber("/camera/color/image_raw/compressed",
            CompressedImage, self.callback,  queue_size = 2)
        self.yolo_sub = rospy.Subscriber("/output/yolo", String, self.callback_yolo, queue_size = 10)
        self.yolo_q = []

    def callback_yolo(self, data):
        self.yolo_q.append(data.data)
        #rospy.loginfo(rospy.get_caller_id() + f" yolo: {data.data}" )

    def callback(self, ros_data):
        #### direct conversion to CV2 ####
        np_arr = np.fromstring(ros_data.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) # OpenCV >= 3.0:

        while len(self.yolo_q): 
          det = self.yolo_q.pop().split(',')
          label = f'{det[0]} {float(det[5]):.2f}'
          xyxy = np.asarray(det[1:5], dtype=np.float64, order='C')
          plot_one_box(xyxy, image, label=label, color=(255,0,0), line_thickness=2)
       
        cv2.imshow('cv_img', image)
        cv2.waitKey(2)


def main(args):
    '''Initializes and cleanup ros node'''
    ic = image_feature()
    rospy.init_node('yolo_listener', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down ROS Image feature detector module")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
