#!/usr/bin/env python
import setup_path 
import airsim
#import rospy 
import argparse
import sys
# argparse not working so far. right now the script is not using any rospy stuff

args = sys.argv
print('args: ', args)
#arg_fmt = argparse.RawDescriptionHelpFormatter
#parser = argparse.ArgumentParser(formatter_class=arg_fmt,
#                                 description="lidar arg parser")
#parser.add_argument('--ip', type=str, default='localhost')
#args = parser.parse_args(rospy.myargv()[1:])
#ipaddr = args.ip
ipaddr = "asus1" 
if len(args)>2:
    if args[1]=='--ip':
        ipaddr=args[2]
print('game host ip: ', ipaddr)
client = airsim.MultirotorClient(ip=ipaddr)
client.confirmConnection()
client.enableApiControl(True)

client.armDisarm(True)

landed = client.getMultirotorState().landed_state
print(landed)
if landed == airsim.LandedState.Landed:
    print("taking off...")
    client.takeoffAsync().join()
else:
    print("already flying...")
    client.takeoffAsync().join()
    client.hoverAsync().join()
client.moveToPositionAsync(0, 0, -3, 5).join()
