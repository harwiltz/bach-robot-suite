import pybullet_envs
from pybullet_envs import register

register(
    id='RocketLanderBRSEnv-v0',
    entry_point='brs_envs.martlet9:RocketLanderEnv',
    max_episode_steps=3000,
    reward_threshold=1e5,
)
