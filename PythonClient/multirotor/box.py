import setup_path
import airsim

import sys
import time
import argparse

    # Parse configuration files
parser = argparse.ArgumentParser(description='hello_drone')
parser.add_argument('--ip', type=str, default='localhost') # ip config
# parser.add_argument('--pretrain_num_epochs', type=int, default=15) # how many epoch to pretrain
args                = parser.parse_args()
ipaddr             = args.ip

def printUsage():
   print("Usage: python box.py --ip ipaddr ")

client = airsim.MultirotorClient(ip=ipaddr)
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)
client.takeoffAsync().join()

print("Flying a small square box using moveByVelocityZ")

# AirSim uses NED coordinates so negative axis is up.
# z of -7 is 7 meters above the original launch point.
z = -7

# Fly given velocity vector for 5 seconds
duration = 10
speed = 1
delay = duration * speed

# using airsim.DrivetrainType.MaxDegreeOfFreedom means we can control the drone yaw independently
# from the direction the drone is flying.  I've set values here that make the drone always point inwards
# towards the inside of the box (which would be handy if you are building a 3d scan of an object in the real world).
vx = speed
vy = 0
print("moving by velocity vx=" + str(vx) + ", vy=" + str(vy) + ", yaw=90")
client.moveByVelocityZAsync(vx,vy,z,duration, airsim.DrivetrainType.MaxDegreeOfFreedom, airsim.YawMode(False, 0)).join()
time.sleep(delay)
vx = 0
vy = speed
print("moving by velocity vx=" + str(vx) + ", vy=" + str(vy)+ ", yaw=180")
client.moveByVelocityZAsync(vx,vy,z,duration, airsim.DrivetrainType.MaxDegreeOfFreedom, airsim.YawMode(False, 90)).join()
time.sleep(delay)
vx =0# -speed
vy = 0
print("moving by velocity vx=" + str(vx) + ", vy=" + str(vy)+ ", yaw=270")
client.moveByVelocityZAsync(vx, vy, z,duration, airsim.DrivetrainType.MaxDegreeOfFreedom, airsim.YawMode(False, 0)).join()
time.sleep(delay)
vx = 0
vy = -speed
print("moving by velocity vx=" + str(vx) + ", vy=" + str(vy) + ", yaw=0")
client.moveByVelocityZAsync(vx, vy,z,duration, airsim.DrivetrainType.MaxDegreeOfFreedom, airsim.YawMode(False, 0)).join()
time.sleep(delay)
client.hoverAsync().join()
client.landAsync().join()
