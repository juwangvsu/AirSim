#!/usr/bin/env python
# Python libs
import sys
import time

import argparse
import time
from pathlib import Path

import torch
import torch.backends.cudnn as cudnn
from numpy import random

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
from sensor_msgs.msg import Image

# Read these to install cv_bridge for python3
# https://stackoverflow.com/questions/49221565/unable-to-use-cv-bridge-with-ros-kinetic-and-python3
# https://cyaninfinite.com/ros-cv-bridge-with-python-3/
from cv_bridge import CvBridge, CvBridgeError

sys.path.append("/home/student/Documents/yolov5") # change it to your yolov5 dir

from utils.torch_utils import select_device, load_classifier, time_synchronized
from utils.plots import plot_one_box
from utils.general import check_img_size, check_requirements, non_max_suppression, apply_classifier, scale_coords, \
    xyxy2xywh, strip_optimizer, set_logging, increment_path
from utils.datasets import LoadStreams, LoadImages, letterbox
from models.experimental import attempt_load


class object_detect:
    def __init__(self):
        '''Initialize ros publisher, ros subscriber'''
        # topic where we publish
        self.image_pub = rospy.Publisher(
            "/output/color/compressed", CompressedImage, queue_size=2)
        self.image_pub1 = rospy.Publisher(
            "/output/depth/compressed", CompressedImage, queue_size=2)
        self.yolo_pub = rospy.Publisher(
            "/output/yolo", String, queue_size=10)
        self.bridge = CvBridge()

        # subscribed Topic
        self.subscriber = rospy.Subscriber(
            "/camera/color/image_raw", Image, self.callback, queue_size=1) # RGB image
        self.subscriber1 = rospy.Subscriber(
            "/camera/depth/image_rect_raw", Image, self.callback1, queue_size=1) # Depth image

        ### yolov5 options ###
        weights, view_img, save_txt, imgsz = opt.weights, opt.view_img, opt.save_txt, opt.img_size

        # Directories
        self.save_dir = Path(increment_path(
            Path(opt.project) / opt.name, exist_ok=opt.exist_ok))  # increment run
        (self.save_dir / 'labels' if save_txt else self.save_dir).mkdir(parents=True,
                                                                        exist_ok=True)  # make dir

        # Initialize
        set_logging()
        self.device = select_device(opt.device)
        self.half = self.device.type != 'cpu'  # half precision only supported on CUDA

        # Load model
        self.model = attempt_load(
            weights, map_location=self.device)  # load FP32 model
        self.imgsz = check_img_size(
            imgsz, s=self.model.stride.max())  # check img_size
        if self.half:
            self.model.half()  # to FP16

        # Second-stage classifier
        self.classify = False
        if self.classify:
            self.modelc = load_classifier(name='resnet101', n=2)  # initialize
            self.modelc.load_state_dict(torch.load(
                'weights/resnet101.pt', map_location=device)['model']).to(device).eval()

        # Set Dataloader
        self.vid_path, self.vid_writer = None, None
        self.view_img = False
        self.save_img = False
        cudnn.benchmark = True  # set True to speed up constant image size inference

        # Get names and colors
        self.names = self.model.module.names if hasattr(
            self.model, 'module') else self.model.names
        self.colors = [[random.randint(0, 255)
                        for _ in range(3)] for _ in self.names]

    def callback(self, ros_data):
        '''Callback function of subscribed topic. 
        Here images get converted and features detected'''
        #print('received image of type: "%s"' % ros_data.encoding)

        #### direct conversion to CV2 ####
        #np_arr = np.fromstring(ros_data.data, np.uint8)
        #image_np = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
        #image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) # OpenCV >= 3.0:

        try:
            image_np = self.bridge.imgmsg_to_cv2(ros_data, "bgr8")
        except CvBridgeError as e:
            print(e)

        ########################################################
        ####                    yolov5                      ####
        ########################################################
        weights, view_img, save_txt, imgsz = opt.weights, opt.view_img, opt.save_txt, opt.img_size

        # Letterbox
        img = [letterbox(x, new_shape=imgsz, auto=False)[0]
               for x in [image_np]]
        # Stack
        img = np.stack(img, 0)
        # Convert
        #img = img[:, :, :, ::-1].transpose(0, 3, 1, 2)  # BGR to RGB, to bsx3x416x416
        img = img.transpose(0, 3, 1, 2)  # RGB, to bsx3x416x416
        img = np.ascontiguousarray(img)

        img = torch.from_numpy(img).to(self.device)
        img = img.half() if self.half else img.float()  # uint8 to fp16/32
        img /= 255.0  # 0 - 255 to 0.0 - 1.0
        if img.ndimension() == 3:
            img = img.unsqueeze(0)

        # Inference
        t1 = time_synchronized()
        pred = self.model(img, augment=opt.augment)[0]

        # Apply NMS
        pred = non_max_suppression(
            pred, opt.conf_thres, opt.iou_thres, classes=opt.classes, agnostic=opt.agnostic_nms)
        t2 = time_synchronized()

        # Apply Classifier
        if self.classify:
            pred = apply_classifier(pred, self.modelc, img, im0s)

        # Process detections
        for i, det in enumerate(pred):  # detections per image
            p, s, im0, frame = 'ros', '%g: ' % i, image_np.copy(), "rgb"

            p = Path(p)  # to Path
            save_path = str(self.save_dir / p.name)  # img.jpg
            #txt_path = str(save_dir / 'labels' / p.stem) + ('' if dataset.mode == 'image' else f'_{frame}')  # img.txt
            txt_path = str(self.save_dir / 'labels' / p.stem) + \
                ('' if False else f'_{frame}')  # img.txt
            s += '%gx%g ' % img.shape[2:]  # print string
            # normalization gain whwh
            gn = torch.tensor(im0.shape)[[1, 0, 1, 0]]
            if len(det):
                # Rescale boxes from img_size to im0 size
                det[:, :4] = scale_coords(
                    img.shape[2:], det[:, :4], im0.shape).round()

                # Print results
                for c in det[:, -1].unique():
                    n = (det[:, -1] == c).sum()  # detections per class
                    # add to string
                    s += f"{n} {self.names[int(c)]}{'s' * (n > 1)}, "

                # Write results
                for *xyxy, conf, cls in reversed(det):
                    line = (*xyxy, conf)
                    self.yolo_pub.publish(f"{self.names[int(cls)]}," + "%g,%g,%g,%g,%g" % line)

                    if save_txt:  # Write to file
                        xywh = (xyxy2xywh(torch.tensor(xyxy).view(1, 4)
                                          ) / gn).view(-1).tolist()  # normalized xywh
                        # label format
                        line = (
                            cls, *xywh, conf) if opt.save_conf else (cls, *xywh)
                        with open(txt_path + '.txt', 'a') as f:
                            f.write(('%g ' * len(line)).rstrip() % line + '\n')

                    if True or self.save_img or self.view_img:  # Add bbox to image
                        label = f'{self.names[int(cls)]} {conf:.2f}'
                        plot_one_box(xyxy, im0, label=label,
                                     color=self.colors[int(cls)], line_thickness=3)

            # Print time (inference + NMS)
            print(f'{s}Done. ({t2 - t1:.3f}s)')

            # Stream results
            if self.view_img:
                cv2.imshow(str(p), im0)
                cv2.waitKey(2)

            # Save results (image with detections)
            if self.save_img:
                if dataset.mode == 'image':
                    cv2.imwrite(save_path, im0)
                else:  # 'video'
                    if vid_path != save_path:  # new video
                        vid_path = save_path
                        if isinstance(vid_writer, cv2.VideoWriter):
                            vid_writer.release()  # release previous video writer

                        fourcc = 'mp4v'  # output video codec
                        fps = vid_cap.get(cv2.CAP_PROP_FPS)
                        w = int(vid_cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                        h = int(vid_cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                        vid_writer = cv2.VideoWriter(
                            save_path, cv2.VideoWriter_fourcc(*fourcc), fps, (w, h))
                    vid_writer.write(im0)

        #### Create CompressedIamge ####
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', im0)[1]).tobytes()
        # Publish new image
        self.image_pub.publish(msg)

        #self.subscriber.unregister()

    def callback1(self, ros_data): 
        '''Callback function of subscribed topic. 
        Here images get converted and features detected'''
        #print('received image of type: "%s"' % ros_data.encoding)

        #### direct conversion to CV2 ####
        #np_arr = np.fromstring(ros_data.data, np.uint8)
        #image_np = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
        #image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) # OpenCV >= 3.0:

        #try:
        #    image_np = self.bridge.imgmsg_to_cv2(ros_data, "u16")
        #except CvBridgeError as e:
        #    print(e)
        image_np = self.bridge.imgmsg_to_cv2(ros_data)
        rmax = np.quantile(image_np, 0.95)
        image_u8 = image_np.copy()
        image_u8[image_u8 > rmax] = rmax
        image_u8 = (image_u8*(256./rmax)).astype('uint8')
        im_color = cv2.applyColorMap(image_u8, cv2.COLORMAP_JET)

        #### Create CompressedIamge ####
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', im_color)[1]).tobytes()
        # Publish new image
        self.image_pub1.publish(msg)

        #self.subscriber.unregister()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--weights', nargs='+', type=str,
                        default='yolov5s.pt', help='model.pt path(s)')
    parser.add_argument('--img-size', type=int, default=640,
                        help='inference size (pixels)')
    parser.add_argument('--conf-thres', type=float,
                        default=0.25, help='object confidence threshold')
    parser.add_argument('--iou-thres', type=float,
                        default=0.45, help='IOU threshold for NMS')
    parser.add_argument('--device', default='',
                        help='cuda device, i.e. 0 or 0,1,2,3 or cpu')
    parser.add_argument('--view-img', action='store_true',
                        help='display results')
    parser.add_argument('--save-txt', action='store_true',
                        help='save results to *.txt')
    parser.add_argument('--save-conf', action='store_true',
                        help='save confidences in --save-txt labels')
    parser.add_argument('--classes', nargs='+', type=int,
                        help='filter by class: --class 0, or --class 0 2 3')
    parser.add_argument('--agnostic-nms', action='store_true',
                        help='class-agnostic NMS')
    parser.add_argument('--augment', action='store_true',
                        help='augmented inference')
    parser.add_argument('--update', action='store_true',
                        help='update all models')
    parser.add_argument('--project', default='runs/detect',
                        help='save results to project/name')
    parser.add_argument('--name', default='exp',
                        help='save results to project/name')
    parser.add_argument('--exist-ok', action='store_true',
                        help='existing project/name ok, do not increment')
    for indx in range(len(sys.argv)):
        if sys.argv[indx].find('--')==0:
            print('break ', indx)
            break
    print(indx, sys.argv[indx:])

    if not sys.argv[indx].find('--')==0: # no -- in cmd line and indx is end of argv list, make it +1 so empty
        indx = indx+1

    opt = parser.parse_args(sys.argv[indx:])
    print(opt)

    '''Initializes and cleanup ros node'''
    rospy.init_node('object_detect', anonymous=True)
    od = object_detect()

    with torch.no_grad():
        if opt.update:  # update all models (to fix SourceChangeWarning)
            for opt.weights in ['yolov5s.pt', 'yolov5m.pt', 'yolov5l.pt', 'yolov5x.pt']:
                strip_optimizer(opt.weights)
    

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down ROS Image feature detector module")

    cv2.destroyAllWindows()
