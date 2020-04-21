import gym
import hydra
import numpy as np
import time

from sac import SACAgent

import brs_envs

@hydra.main(config_path='config-rocket-lander.yaml', strict=False)
def main(cfg):
    agent = hydra.utils.instantiate(cfg.agent)
    agent.train(cfg.training.steps)
    env_kwargs = cfg.agent.params.env_kwargs
    env_kwargs['render'] = True
    agent._env_fn = lambda: gym.make(cfg.agent.params.env, **env_kwargs)
    while True:
        input("Press ENTER to watch simulation...")
        results = agent.rollout(render=False)
        print("Episode score: {}".format(results))

if __name__ == "__main__":
    main()
