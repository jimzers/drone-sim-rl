import airsim

import os
import pprint

import cv2
import numpy as np

client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

state = client.getMultirotorState()
s = pprint.pformat(state)
print("state: %s" % s)

airsim.wait_key('type something to move')
client.takeoffAsync().join()

client.moveToPositionAsync(-40, 40, -40, 10).join()

client.hoverAsync().join()

state = client.getMultirotorState()
s = pprint.pformat(state)
print("state: %s" % s)


airsim.wait_key('press again to reset')

# exit protocol
client.armDisarm(False)
client.reset()

client.enableApiControl(False)
