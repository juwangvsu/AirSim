#!/usr/bin/env python
import setup_path 
import airsim
import sys

args=sys.argv
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
if landed == airsim.LandedState.Landed:
    print("already landed...")
    client.landAsync().join()
else:
    print("landing...")
    client.landAsync().join()

client.armDisarm(False)
client.enableApiControl(False)
