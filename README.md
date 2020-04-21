# Bach Robot Suite

This repo contains some novel robot simulators with PyBullet with Gym environments for training RL
agents.

## Usage

First install the dependencies for the suite,

```bash
pip install -r --user requirements/requirements.txt
```
The training examples in the `examples/` directory use
[Hydra](https://github.com/facebookresearch/hydra) and [my implementation of
SAC](https://github.com/harwiltz/sac). To run these training examples, you can install their
dependencies with

```bash
pip install -r --user requirements/examples.txt
```

Then, you may install the robot suite with

```bash
python setup.py install --user
```

This installs two packages, `brs_data` and `brs_envs`; the former containing the robot definitions,
and the latter containing the Gym environments. If you simply wish to use the environments (as
opposed to developing new ones), you only need to concern yourself with the `brs_envs` package.

### Training

You can execute a training run with

```bash
python examples/rocket_lander.py
```

The `examples/rocket_lander.py` script is configured by `examples/config-rocket-lander.yaml` using
Hydra. You may tune some hyperparameters there or via the command line, for example

```bash
python examples/rocket_lander.py training.steps=50000 \
                                 agent.params.max_replay_capacity=100000 \
                                 agent.params.env_kwargs.max_pitch_offset=0.1
```
