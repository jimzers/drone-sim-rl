import time

import gym
from PIL import Image
from gym import spaces

import numpy as np

import airsim

# Continous Action Space
class DroneEnvContinuous(gym.Env):
    def __init__(self, img_height, img_width, n_channels=1,
                 x_range_lim=1, y_range_lim=1, z_range_lim=1,
                 initX=0, initY=-20, initZ=0,
                 action_duration=5, collision_penalty=-100):
        super(DroneEnvContinuous, self).__init__()

        self.action_duration = action_duration
        self.collision_penalty = collision_penalty

        print('before action space')
        self.action_space = spaces.Tuple((
            spaces.Box(low=-x_range_lim, high=x_range_lim, shape=[1]),
            spaces.Box(low=-y_range_lim, high=y_range_lim, shape=[1]),
            spaces.Box(low=-z_range_lim, high=z_range_lim, shape=[1])
        ))
        # todo: normalize observation space
        self.observation_space = spaces.Box(low=0, high=255,
                                            shape=(img_height, img_width, n_channels), dtype=np.uint8)

        self.initX = initX
        self.initY = initY
        self.initZ = initZ

        # todo: init airsim client
        print('milestone TWO')
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()
        self.client.enableApiControl(True)
        self.client.armDisarm(True)
        print('pogchampion')

    def teleport_drone(self, x, y, z):
        # teleport the drone
        pose = self.client.simGetVehiclePose()
        # set everything to zero first (this keeps the same data type)
        pose.position.x_val -= pose.position.x_val
        pose.position.y_val -= pose.position.y_val
        pose.position.z_val -= pose.position.z_val
        # set everything to the new position
        pose.position.x_val += x
        pose.position.y_val += y
        pose.position.z_val += z

        self.client.simSetVehiclePose(pose, True)

    def compute_reward(self, quad_state, quad_vel, collision_info):
        if collision_info.has_collided:
            print('NO REWARD FOR U LOL')
            reward = self.collision_penalty
        else:
            # implement your own reward function here
            # sample reward function: radius of value
            # reward a high speed but must stay within a certain x and z range
            quad_pt = np.array(list((quad_state.x_val, quad_state.y_val, quad_state.z_val)))

            y_sweetspot = 40
            x_edge = 40
            z_edge = 40
            y_reward = 5

            height_r = y_reward - (quad_pt[1] - y_sweetspot)**2
            x_reward = 0
            z_reward = 0
            if -x_edge < quad_pt[0] < x_edge:
                x_reward = 2
            if -z_edge < quad_pt[2] < z_edge:
                z_reward = 2

            reward = height_r + x_reward + z_reward
            print(reward)

        return reward

    def transform_input(self, responses):
        img1d = np.array(responses[0].image_data_float, dtype=np.float)
        img1d = 255 / np.maximum(np.ones(img1d.size), img1d)
        img2d = np.reshape(img1d, (responses[0].height, responses[0].width))

        image = Image.fromarray(img2d)
        im_final = np.array(image.resize((84, 84)).convert('L'))
        im_final = np.expand_dims(im_final, axis=-1)
        # print("TRANSFORMING INPUT")
        # print(im_final.shape)

        return im_final

    def step(self, action):
        print('stepping function called')
        # print(action)
        # print(type(action))
        # print(len(action))
        # print(action[0][0])
        # print(type(action[0][0]))
        quad_vel = self.client.getMultirotorState().kinematics_estimated.linear_velocity
        self.client.moveByVelocityAsync(quad_vel.x_val + action[0][0], quad_vel.y_val + action[1][0],
                                        quad_vel.z_val + action[2][0], self.action_duration).join()
        time.sleep(0.5)

        quad_state = self.client.getMultirotorState().kinematics_estimated.position
        quad_vel = self.client.getMultirotorState().kinematics_estimated.linear_velocity
        collision_info = self.client.simGetCollisionInfo()
        reward = self.compute_reward(quad_state, quad_vel, collision_info)

        observation = self.client.simGetImages([airsim.ImageRequest(3, airsim.ImageType.DepthPerspective, True, False)])
        observation = self.transform_input(observation)
        done = collision_info.has_collided
        info = self.client.getMultirotorState()
        info = {"multirotor_state": info}

        # print("============= STEP FUNCTION OBSERVATION: ===================")
        # print(observation.shape)
        # print("===============TYPE==================")
        # print(observation.dtype)
        # print("=================================================================")
        #
        # print("================================INFO FUNCTION THINGY=================================")
        # print({"multirotor_state": info})

        return observation, reward, done, info

    def reset(self):
        print('reset function called')
        self.client.reset()
        time.sleep(0.2)
        self.client.enableApiControl(True)
        self.client.armDisarm(True)

        self.teleport_drone(self.initX, self.initY, self.initZ)
        time.sleep(0.1)
        self.client.takeoffAsync().join()
        time.sleep(0.5)

        observation = self.client.simGetImages([airsim.ImageRequest(3, airsim.ImageType.DepthPerspective, True, False)])
        observation = self.transform_input(observation)

        return observation

    def close(self):
        print('closing fn called')
        # exit protocol
        self.client.armDisarm(False)
        self.client.reset()
        self.client.enableApiControl(False)

from stable_baselines.common.env_checker import check_env
env = DroneEnvContinuous(84, 84)
check_env(env)