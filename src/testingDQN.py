import math
import time
import os

import numpy as np
from PIL import Image

import torch

import airsim

from stable_baselines import DQN
from stable_baselines.deepq.policies import CnnPolicy

from AirSimGym.envs.gym_drone_DQN import DroneEnvDQN
# from stable_baselines.common.env_checker import check_env
# env = DroneEnvDQN(255, 255, 3)
# check_env(env)


env = DroneEnvDQN(84, 84, 1)
model = DQN(CnnPolicy, env, verbose=1)

model.learn(total_timesteps=25000)
model.save('deepq_drone')
