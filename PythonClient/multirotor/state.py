import setup_path
import airsim
import pprint

def print_state():
    print("===============================================================")
    state = client.getMultirotorState()
    print("state: %s" % pprint.pformat(state))
    return state


client = airsim.MultirotorClient(ip="msi")
client.confirmConnection()
state = print_state()
client.enableApiControl(True)

if state.ready:
    print("drone is ready!")
else:
    print("drone is not ready!")
if state.ready_message:
    print(state.ready_message)
