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
parser.add_argument('--yaw', type=str, default='0') # ip config
parser.add_argument('--takeoff', dest='takeoff', action='store_true')
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
print("takeoff: ", args.takeoff )
if args.takeoff:
    print('do takeoff')
#airsim.wait_key('Press any key to takeoff')
   # client.takeoffAsync().join()

#client.moveByAngleZAsync(pitch=0, roll=0, z, 90 , 2).join()
client.rotateToYawAsync(yaw=float(args.yaw), margin = 5).join()


