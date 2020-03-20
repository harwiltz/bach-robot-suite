import gym
import numpy as np
import time

import brs_envs

env = gym.make('RocketLanderBRSEnv-v0', render=True)
env.reset()

zero_action = np.zeros(env.action_space.shape)

for _ in range(5000):
    time.sleep(1/60)
    s, r, done, _ = env.step(zero_action)
    if done:
        print("Reward in final frame: {}".format(r))
        break
