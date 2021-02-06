import setup_path 
import airsim

import numpy as np
import os
import tempfile
import pprint
import cv2
import random
import math
import shutil
import argparse
########################################################################
#input is the depth 2d np array (h,w) float, fov
#
def savePointCloud(fileName, image,fov):
   color = (0,255,0)
   rgb = "%d %d %d" % color
   f = open(fileName, "w")
   print('image np array shape: ', image.shape)
   height = image.shape[0]
   width = image.shape[1]
   for y in range(image.shape[0]):
     for x in range(image.shape[1]):
        z_cam = image[y,x]
        if (math.isinf(z_cam) or math.isnan(z_cam)):
          # skip it
          None
        else: 
          y_normalized_camplan = (y-height/2)/(height/2) 
          y_cam = y_normalized_camplan * z_cam
          x_normalized_camplan = (x-width/2)/(width/2) 
          x_cam = x_normalized_camplan * z_cam
          f.write("%f %f %f %s\n" % (z_cam, x_cam, y_cam, rgb))# cam to world frame : z_cam is x_world, y_cam is z_world, x_cam is y_world
   f.close()
################################################################################
# image is np array of shape(h,w), float
def write_pfm_txt(file, image, scale=1):
    """ Write a pfm file """
    file = open(file, 'wb')

    color = None

    if image.dtype.name != 'float32':
        raise Exception('Image dtype must be float32.')

    if len(image.shape) == 3 and image.shape[2] == 3: # color image
        color = True
    elif len(image.shape) == 2 or len(image.shape) == 3 and image.shape[2] == 1: # greyscale
        color = False
    else:
        raise Exception('Image must have H x W x 3, H x W x 1 or H x W dimensions.')


    temp_str = '%d %d\n' % (image.shape[1], image.shape[0])
    file.write(bytes(temp_str, 'UTF-8'))


    temp_str = '%f\n' % scale
    file.write(bytes(temp_str, 'UTF-8'))

    #image.tofile(file)
#    np.savetxt('test.out', image, delimiter=',')
    np.savetxt('test_depth.txt', image, fmt='%3.3f')
#######################################################################################

    # Parse configuration files
parser = argparse.ArgumentParser(description='hello_drone')
parser.add_argument('--ip', type=str, default='localhost') # ip config
# parser.add_argument('--pretrain_num_epochs', type=int, default=15) # how many epoch to pretrain
args                = parser.parse_args()
ipaddr             = args.ip

# connect to the AirSim simulator
client = airsim.MultirotorClient(ip=ipaddr)
#client = airsim.MultirotorClient(ip="192.168.86.61")
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)
state = client.getMultirotorState()
s = pprint.pformat(state)
print("state: %s" % s)

imu_data = client.getImuData()
s = pprint.pformat(imu_data)
print("imu_data: %s" % s)

barometer_data = client.getBarometerData()
s = pprint.pformat(barometer_data)
print("barometer_data: %s" % s)

magnetometer_data = client.getMagnetometerData()
s = pprint.pformat(magnetometer_data)
print("magnetometer_data: %s" % s)

gps_data = client.getGpsData()
s = pprint.pformat(gps_data)
print("gps_data: %s" % s)

#airsim.wait_key('Press any key to takeoff')
#client.takeoffAsync().join()

state = client.getMultirotorState()
print("state: %s" % pprint.pformat(state))

#airsim.wait_key('Press any key to move vehicle to (-10, 10, -10) at 5 m/s')
#client.moveToPositionAsync(-10, 10, -10, 5).join()

#client.hoverAsync().join()

state = client.getMultirotorState()
print("state: %s" % pprint.pformat(state))
caminfo = client.simGetCameraInfo("0")
print("cam info: ", caminfo, caminfo.fov, type(caminfo.fov))

airsim.wait_key('Press any key to take images')
# get camera images from the car
responses = client.simGetImages([
    airsim.ImageRequest("0", airsim.ImageType.DepthVis, True),  #depth visualization image
    airsim.ImageRequest("1", airsim.ImageType.DepthPerspective, True), #depth in perspective projection
    airsim.ImageRequest("1", airsim.ImageType.DepthPlanner,True), #scene vision image in png format
	    airsim.ImageRequest("1", airsim.ImageType.DepthPlanner), #scene vision image in png format
    airsim.ImageRequest("1", airsim.ImageType.Scene, False, False)
])  #scene vision image in uncompressed RGBA array
print('Retrieved images: %d' % len(responses))

tmp_dir = os.path.join(tempfile.gettempdir(), "airsim_drone")
print ("Saving images to %s" % tmp_dir)
try:
    os.makedirs(tmp_dir)
except OSError:
    if not os.path.isdir(tmp_dir):
        raise

for idx, response in enumerate(responses):

    filename = os.path.join(".", str(idx))

    if response.pixels_as_float:
        print("Type %d, size %d" % (response.image_type, len(response.image_data_float)))
        airsim.write_pfm(os.path.normpath(filename + '.pfm'), airsim.get_pfm_array(response))
        write_pfm_txt(os.path.normpath(filename + '.txt'), airsim.get_pfm_array(response))
        savePointCloud('depth_cloud'+str(idx)+'.asc', airsim.get_pfm_array(response), caminfo.fov)
    elif response.compress: #png format
        print("Type %d, size %d" % (response.image_type, len(response.image_data_uint8)))
        airsim.write_file(os.path.normpath(filename + '.png'), response.image_data_uint8)
    else: #uncompressed array
        print("Type %d, size %d" % (response.image_type, len(response.image_data_uint8)))
        img1d = np.fromstring(response.image_data_uint8, dtype=np.uint8) # get numpy array
        img_rgb = img1d.reshape(response.height, response.width, 3) # reshape array to 4 channel image array H X W X 3
        cv2.imwrite(os.path.normpath(filename + '.png'), img_rgb) # write to png

airsim.wait_key('Press any key to reset to original state')

client.armDisarm(False)
client.reset()

# that's enough fun for now. let's quit cleanly
client.enableApiControl(False)
