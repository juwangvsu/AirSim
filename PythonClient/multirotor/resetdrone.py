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
import time
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

client.reset()

# that's enough fun for now. let's quit cleanly
client.enableApiControl(False)
