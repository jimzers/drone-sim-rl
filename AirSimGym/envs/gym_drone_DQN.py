import time
import math

import gym
from PIL import Image
from gym import spaces

import numpy as np

import airsim


# Discrete Action Space, meant for interface with DQN
class DroneEnvDQN(gym.Env):
    def __init__(self, img_height, img_width, n_channels=1, n_actions=7):
        super(DroneEnvDQN, self).__init__()

        self.action_space = spaces.Discrete(n_actions)
        # todo: normalize observation space
        self.observation_space = spaces.Box(low=0, high=255,
                                            shape= (img_height, img_width, n_channels), dtype=np.uint8)

        # todo: init airsim client
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()
        self.client.enableApiControl(True)
        self.client.armDisarm(True)

        self.initX = -.55265
        self.initY = -31.9786
        self.initZ = -19.0225

        self.client.takeoffAsync().join()
        self.client.moveToPositionAsync(self.initX, self.initY, self.initZ, 5).join()
        self.client.moveByVelocityAsync(1, -0.67, -0.8, 5).join()
        time.sleep(0.5)

    # static fn to interpret action
    def interpret_action(self, action):
        scaling_factor = 0.25
        if action == 0:
            quad_offset = (0, 0, 0)
        elif action == 1:
            quad_offset = (scaling_factor, 0, 0)
        elif action == 2:
            quad_offset = (0, scaling_factor, 0)
        elif action == 3:
            quad_offset = (0, 0, scaling_factor)
        elif action == 4:
            quad_offset = (-scaling_factor, 0, 0)
        elif action == 5:
            quad_offset = (0, -scaling_factor, 0)
        elif action == 6:
            quad_offset = (0, 0, -scaling_factor)
        else:
            raise Exception('bruh argument not in bounds: ', action)

        return quad_offset

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

    def compute_reward(self, quad_state, quad_vel, collision_info):
        thresh_dist = 7
        beta = 1

        z = -10
        # power lines LOL
        pts = [np.array([-.55265, -31.9786, -19.0225]), np.array([48.59735, -63.3286, -60.07256]),
               np.array([193.5974, -55.0786, -46.32256]), np.array([369.2474, 35.32137, -62.5725]),
               np.array([541.3474, 143.6714, -32.07256])]

        quad_pt = np.array(list((quad_state.x_val, quad_state.y_val, quad_state.z_val)))

        if collision_info.has_collided:
            reward = -100
        else:
            dist = 10000000
            for i in range(0, len(pts) - 1):
                dist = min(dist, np.linalg.norm(np.cross((quad_pt - pts[i]), (quad_pt - pts[i + 1]))) / np.linalg.norm(
                    pts[i] - pts[i + 1]))

            # print(dist)
            if dist > thresh_dist:
                reward = -10
            else:
                reward_dist = (math.exp(-beta * dist) - 0.5)
                reward_speed = (np.linalg.norm([quad_vel.x_val, quad_vel.y_val, quad_vel.z_val]) - 0.5)
                reward = reward_dist + reward_speed

        return reward

    def step(self, action):
        quad_offset = self.interpret_action(action)
        quad_vel = self.client.getMultirotorState().kinematics_estimated.linear_velocity
        self.client.moveByVelocityAsync(quad_vel.x_val + quad_offset[0], quad_vel.y_val + quad_offset[1],
                                   quad_vel.z_val + quad_offset[2], 5).join()
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
        # self.client.reset()
        self.client.moveToPositionAsync(self.initX, self.initY, self.initZ, 5).join()
        self.client.moveByVelocityAsync(1, -0.67, -0.8, 5).join()
        time.sleep(0.5)
        observation = self.client.simGetImages([airsim.ImageRequest(3, airsim.ImageType.DepthPerspective, True, False)])
        observation = self.transform_input(observation)
        # print("============= RESET FUNCTION OBSERVATION: ===================")
        # print(observation.shape)
        # print("===============TYPE==================")
        # print(observation.dtype)
        # print("=================================================================")
        return observation

    """
    def render(self):
        ...
        return observation
    """

    def close(self):
        # exit protocol
        self.client.armDisarm(False)
        self.client.reset()

        self.client.enableApiControl(False)

# # test the env
# env = DroneEnvDQN(84, 84, 3)
# for epoch in range(5):
#     print('resetting environment')
#     s = env.reset()
#     for t in range(5):
#         print('next step')
#         s_n, r, d, i = env.step(action=env.action_space.sample())
#
# print('finished')
# env.close()


from stable_baselines.common.env_checker import check_env
env = DroneEnvDQN(84, 84)
check_env(env)