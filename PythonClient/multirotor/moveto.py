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

    # Parse configuration files
parser = argparse.ArgumentParser(description='hello_drone')
parser.add_argument('--ip', type=str, default='localhost') # ip config
parser.add_argument('--xyz', type=str, default='3,-3,-1') # ip config
# parser.add_argument('--pretrain_num_epochs', type=int, default=15) # how many epoch to pretrain
args                = parser.parse_args()
ipaddr             = args.ip

# connect to the AirSim simulator
client = airsim.MultirotorClient(ip=ipaddr)
#client = airsim.MultirotorClient(ip=ipaddr, port=12089)
#client = airsim.MultirotorClient(ip="192.168.86.61")
#client.confirmConnection()
#client.enableApiControl(True)
#client.armDisarm(True)
state = client.getMultirotorState()
s = pprint.pformat(state)
print("state: %s" % s)


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

airsim.wait_key('Press any key to move vehicle to (-10, 10, -10) at 5 m/s')
print("xyz ", args.xyz.split(','))
x,y,z = args.xyz.split(',')

client.moveToPositionAsync(float(x), float(y), float(z),1).join()
#client.moveToPositionAsync(3,-3,-1,1).join()

#client.hoverAsync().join()

state = client.getMultirotorState()
print("state: %s" % pprint.pformat(state))

