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
# parser.add_argument('--pretrain_num_epochs', type=int, default=15) # how many epoch to pretrain
args                = parser.parse_args()
ipaddr             = args.ip

# connect to the AirSim simulator
client = airsim.MultirotorClient(ip=ipaddr)
#client = airsim.MultirotorClient(ip=ipaddr, port=12089)
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

airsim.wait_key('Press any key to move vehicle to (-10, 10, -10) at 5 m/s')
client.moveToPositionAsync(-60, 40, -10, 5).join()

client.hoverAsync().join()

state = client.getMultirotorState()
print("state: %s" % pprint.pformat(state))

