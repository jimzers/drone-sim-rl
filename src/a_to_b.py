import airsim


client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)



# def teleport_drone(x, y, z):
#     # teleport the drone
#     pose = client.simGetVehiclePose()
#     # set everything to zero first (this keeps the same data type)
#     pose.position.x_val -= pose.position.x_val
#     pose.position.y_val -= pose.position.y_val
#     pose.position.z_val -= pose.position.z_val
#     # set everything to the new position
#     pose.position.x_val += x
#     pose.position.y_val += y
#     pose.position.z_val += z
#
#     client.simSetVehiclePose(pose, True)
#
#
# teleport_drone(0, 0, 400)

client.takeoffAsync().join()



# observation = client.simGetImages([airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)])
# obs = observation[0]

obs = client.simGetImage("0", airsim.ImageType.Scene)
with open('../images/view0.png', 'wb') as f:
    f.write(obs)

# quad_vel = client.getMultirotorState().kinematics_estimated.linear_velocity
quad_offset = (1, 0, -0.5)
client.moveByVelocityAsync(quad_offset[0], quad_offset[1],
                                   quad_offset[2], 5).join()

airsim.time.sleep(4.2)
obs = client.simGetImage("0", airsim.ImageType.Scene)
with open('../images/view1.png', 'wb') as f:
    f.write(obs)


# client.moveByVelocityAsync(quad_vel.x_val + quad_offset[0], quad_vel.y_val + quad_offset[1],
#                                    quad_vel.z_val + quad_offset[2], 1).join()

quad_offset = (0, 1, 0)
client.moveByVelocityAsync(quad_offset[0], quad_offset[1],
                                   quad_offset[2], 5).join()
airsim.time.sleep(4.2)
obs = client.simGetImage("0", airsim.ImageType.Scene)
with open('../images/view2.png', 'wb') as f:
    f.write(obs)

quad_offset = (-1, 0, 0)
client.moveByVelocityAsync(quad_offset[0], quad_offset[1],
                                   quad_offset[2], 5).join()

airsim.time.sleep(4.2)
obs = client.simGetImage("0", airsim.ImageType.Scene)
with open('../images/view3.png', 'wb') as f:
    f.write(obs)

quad_offset = (0, -1, 0)
client.moveByVelocityAsync(quad_offset[0], quad_offset[1],
                                   quad_offset[2], 5).join()
airsim.time.sleep(4.2)
obs = client.simGetImage("0", airsim.ImageType.Scene)
with open('../images/view4.png', 'wb') as f:
    f.write(obs)

# exit protocol
client.armDisarm(False)
client.reset()

client.enableApiControl(False)
