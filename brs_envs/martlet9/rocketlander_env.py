import numpy as np

from gym import spaces

from brs_envs.base_envs import BaseURDFBulletEnv
from brs_envs.base_envs import parse_collision
from brs_envs.rocket_landing_scene import RocketLandingScene
from brs_envs.martlet9.martlet9_robot import Martlet9Robot

class RocketLanderEnv(BaseURDFBulletEnv):
    LANDING_SPEED_PENALTY = 5
    LANDING_SPEED_SURVIVE_THRESH = 10
    DEATH_PENALTY = 500
    LANDED_SPEED_THRESH = 1e-1
    LANDED_BONUS = DEATH_PENALTY
    ACCURACY_BONUS = DEATH_PENALTY / 10
    HIT_WATER_PENALTY = DEATH_PENALTY
    POSITION_THRESH = 20
    def __init__(self,
                 render=False,
                 gravity=9.8,
                 timestep=1/60,
                 sticky=1,
                 max_lateral_offset=10,
                 max_vertical_offset=10,
                 max_roll_offset=0.5,
                 max_pitch_offset=0.5,
                 max_yaw_offset=0.1,
                 mean_robot_start_height=100):
        BaseURDFBulletEnv.__init__(self, render)
        self._feet_landed = set()
        self._gravity = gravity
        self._timestep = timestep
        self._sticky = sticky
        self._max_lateral_offset = max_lateral_offset
        self._max_vertical_offset = max_vertical_offset
        self._max_roll_offset = max_roll_offset
        self._max_pitch_offset = max_pitch_offset
        self._max_yaw_offset = max_yaw_offset
        self._mean_robot_start_height = mean_robot_start_height
        self.observation_space = Martlet9Robot.observation_space
        self.action_space = Martlet9Robot.action_space

    def step(self, a):
        control_cost = self.robot.applyControls(self.p, a)
        self.scene.step()
        state = self.robot.getState(self.p)
        if self.renderable:
            self.moveCamera(state)
            self.drawArtifacts(a)

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
            return RocketLanderEnv.HIT_WATER_PENALTY + RocketLanderEnv.DEATH_PENALTY, True
        num_landed_feet = 0
        new_landed_feet = 0
        allowable_contacts = [self.robot.foot1_link, self.robot.foot2_link, self.robot.foot3_link]
        feet_landed = set()
        prev_state_desc = Martlet9Robot.describeState(prev_state)
        state_desc = Martlet9Robot.describeState(state)
        if state_desc['position'][-1] >= self._mean_robot_start_height + self._max_vertical_offset:
            return RocketLanderEnv.DEATH_PENALTY, True
        lateral_dist_to_center = np.linalg.norm(state_desc['position'][:2])
        if lateral_dist_to_center > RocketLanderEnv.POSITION_THRESH:
            return RocketLanderEnv.DEATH_PENALTY, True
        for collision in map(parse_collision, pad_col):
            other_link = collision['linkIndexB']
            if other_link not in allowable_contacts:
                return RocketLanderEnv.DEATH_PENALTY, True
            num_landed_feet += 1
            feet_landed.add(other_link)
            if other_link not in self._feet_landed:
                new_landed_feet += 1
            self._feet_landed = feet_landed
        if num_landed_feet == 0: # No collisions, no penalties
            return 0.00 * np.linalg.norm(state_desc['position']), False
        speed = np.linalg.norm(state_desc['velocity'])
        prev_speed = np.linalg.norm(prev_state_desc['velocity'])
        if max(speed, prev_speed) > RocketLanderEnv.LANDING_SPEED_SURVIVE_THRESH:
            speed_overshoot = max(speed, prev_speed) - RocketLanderEnv.LANDING_SPEED_SURVIVE_THRESH
            speed_penalty = RocketLanderEnv.LANDING_SPEED_PENALTY * (speed_overshoot**2)
            center_bonus = max(1, 10 - lateral_dist_to_center) * RocketLanderEnv.ACCURACY_BONUS
            print("Landed too fast! Speed was {}".format(max(speed, prev_speed)))
            return RocketLanderEnv.DEATH_PENALTY + speed_penalty - center_bonus, True
        landing_cost = max(speed, prev_speed) * RocketLanderEnv.LANDING_SPEED_PENALTY * min(1, new_landed_feet)
        if (num_landed_feet < 3) or (speed > RocketLanderEnv.LANDED_SPEED_THRESH):
#            return landing_cost, False
            return 0, False
        print("Smooth landing!")
        return landing_cost - RocketLanderEnv.LANDED_BONUS, True

    def reset(self):
        state = BaseURDFBulletEnv.reset(self)
        self.robot.addToScene(self.scene, self.robotStartPos(), self.robotStartOri())
        self._prev_state = self.robot.getState(self.p)
        return self._prev_state

    def drawArtifacts(self, control):
        if self.robot.thruster_fire_id is not None:
            r = 1.0
            g = 0.8 * control[0]
            b = 0.3
            a = min(1.0, 0.9 * control[0])
            self.p.changeVisualShape(self.robot.uid,
                                     self.robot.thruster_fire_id,
                                     rgbaColor=[r, g, b, a])
        if self.robot.steer_smoke_id is not None:
            r = 0.4
            g = 0.4
            b = 0.4
            a = min(1.0, 0.2 * control[2])
            self.p.changeVisualShape(self.robot.uid,
                                     self.robot.steer_smoke_id,
                                     rgbaColor=[r, g, b, a])

    def moveCamera(self, state):
        target = state[:3]
        ori = self.p.getEulerFromQuaternion(state[3:7])
        yaw = 20
        pitch = state[2] / 100
        distance = 0.3 * state[2] + 50
        self.p.resetDebugVisualizerCamera(distance, yaw, pitch, target)

    def initializeScene(self):
        return RocketLandingScene(self.p, gravity=self._gravity, timestep=self._timestep, sticky=self._sticky)

    def initializeRobot(self):
        return Martlet9Robot()

    def robotStartPos(self):
        max_lateral_offset = float(self._max_lateral_offset)
        max_vertical_offset = float(self._max_vertical_offset)
        mean_robot_start_height = float(self._mean_robot_start_height)
        x = np.random.uniform(-max_lateral_offset, max_lateral_offset)
        y = np.random.uniform(-max_lateral_offset, max_lateral_offset)
        z = mean_robot_start_height + np.random.uniform(-max_vertical_offset, max_vertical_offset)
        return [x, y, z]

    def robotStartOri(self):
        max_roll_offset = float(self._max_roll_offset)
        max_pitch_offset = float(self._max_pitch_offset)
        max_yaw_offset = float(self._max_yaw_offset)
        roll = np.random.uniform(-max_roll_offset, max_roll_offset)
        pitch = np.random.uniform(-max_pitch_offset, max_pitch_offset)
        yaw = np.random.uniform(-max_yaw_offset, max_yaw_offset)
        return self.p.getQuaternionFromEuler([roll, pitch, yaw])
