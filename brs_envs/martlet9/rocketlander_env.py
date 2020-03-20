import numpy as np

from brs_envs.base_envs import BaseURDFBulletEnv
from brs_envs.base_envs import parse_collision
from brs_envs.rocket_landing_scene import RocketLandingScene
from brs_envs.martlet9.martlet9_robot import Martlet9Robot

class RocketLanderEnv(BaseURDFBulletEnv):
    def __init__(self, render=False):
        RocketLanderEnv.__init__(self, render)
        self.scene = RocketLandingScene()
        self.robot = Martlet9Robot()

    def step(self, a):
        control_cost = self.robot.applyControls(self.scene.p, a)
        self.scene.step()
        state = self.robot.getState()

        # TODO: Check collisions
        collisions_in_water = self.p.getContactPoints(bodyA=self.scene.plane)
        collisions_on_pad = self.p.getContactPoints(bodyA=self.scene.pad)
        collision_cost, done = self.processCollisions(state,
                                                      self._prev_state,
                                                      collisions_in_water,
                                                      collisions_on_pad)
        self._prev_state = state
        return state, -(control_cost + collision_cost), done, {}

    def processCollisions(self, state, prev_state, water_col, pad_col):
        if len(water_col) > 0:
            return 1000, True
        num_stable_feet = 0
        for collision in map(parse_collision, pad_col):

    def reset(self):
        BaseURDFBulletEnv.reset(self)
        self.scene.reset()
        self.robot.addToScene(scene, ROBOT_START_POS, robotStartOri())
        self._prev_state = None

    def robotStartOri(max_roll_offset=0.1, max_pitch_offset=None, max_yaw_offset=None):
        if max_pitch_offset is None:
            max_pitch_offset = max_roll_offset
        if max_yaw_offset is None:
            max_yaw_offset = max_roll_offset
        roll = np.random.uniform(-max_roll_offset, max_roll_offset)
        pitch = np.random.uniform(-max_pitch_offset, max_pitch_offset)
        yaw = np.random.uniform(-max_yaw_offset, max_yaw_offset)
        return [roll, pitch, yaw]
