#!/usr/bin/env python
import setup_path
import airsim
import pprint
import sys
args=sys.argv
ipaddr = "asus1"
if len(args)>2:
    if args[1]=='--ip':
        ipaddr=args[2]
print('game host ip: ', ipaddr)
client = airsim.MultirotorClient(ip=ipaddr)

def print_state():
    print("===============================================================")
    state = client.getMultirotorState()
    print("state: %s" % pprint.pformat(state))
    return state


client.confirmConnection()
state = print_state()
client.enableApiControl(True)

if state.ready:
    print("drone is ready!")
else:
    print("drone is not ready!")
if state.ready_message:
    print(state.ready_message)
