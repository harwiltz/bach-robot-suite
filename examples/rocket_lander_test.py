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

input("Press ENTER to start...")
target = 80
for i in range(1800):
    time.sleep(1/60)
    if s[2] < target:
        a = np.array([min(1.0, (s[2] - target)**2), 0.0, 0.0])
    else:
        a = np.array([0.3, 0.0, 0.0])
    a += np.array([0.0, 0.15 * np.sin(0.1 * i), 0.8])
    s, r, done, _ = env.step(a)
#    vels.append(np.linalg.norm(s[7:10]))
    vels.append(s[2])
#    s, r, done, _ = env.step(env.action_space.sample())
    if done:
        print("Reward in final frame: {}".format(r))
        break

plt.plot(np.arange(len(vels)) / 60, vels)
plt.xlabel('Time (s)')
plt.ylabel('Position (m)')
plt.show()
