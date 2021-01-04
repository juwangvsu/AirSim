#!/usr/bin/env python
import setup_path 
import airsim

client = airsim.MultirotorClient(ip="192.168.86.61")
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
