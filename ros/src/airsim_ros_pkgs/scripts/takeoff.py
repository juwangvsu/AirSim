#!/usr/bin/env python
import setup_path 
import airsim

client = airsim.MultirotorClient(ip="192.168.86.61")
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
client.moveToPositionAsync(0, 0, -10, 5).join()
