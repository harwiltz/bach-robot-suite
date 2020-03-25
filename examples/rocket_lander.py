import gym
import matplotlib.pyplot as plt
import numpy as np
import time

import brs_envs

env = gym.make('RocketLanderBRSEnv-v0',
               render=True,
               max_lateral_offset=0,
               max_pitch_offset=0,
               max_roll_offset=0,
               max_yaw_offset=0,
               mean_robot_start_height=100)
s = env.reset()

zero_action = np.zeros(env.action_space.shape)

vels = []

for _ in range(5000):
    time.sleep(1/60)
    if s[2] < 0:
        a = np.array([1.0, 0.0, 0.0])
    else:
        a = zero_action
    s, r, done, _ = env.step(a)
    vels.append(np.linalg.norm(s[7:10]))
#    s, r, done, _ = env.step(env.action_space.sample())
    if done:
        print("Reward in final frame: {}".format(r))
        break

plt.plot(vels)
plt.show()
