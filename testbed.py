import gym
import brs_envs

env = gym.make('RocketLanderBRSEnv-v0', render=True)
env.reset()
env.close()
