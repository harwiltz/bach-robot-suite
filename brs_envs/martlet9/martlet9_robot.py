import numpy as np
import os

from gym import spaces

import brs_data

from brs_envs.robot import URDFRobot

class Martlet9Robot(URDFRobot):
    MAX_THRUST = 7e6
    MAX_STEER = 10
    MAX_SHOOT = 1e5
    COSTS = np.array([2., 0., 1.]) / np.array([MAX_THRUST, MAX_STEER, MAX_SHOOT])

    observation_space = spaces.Box(-np.inf, np.inf, shape=(20,))
    action_space = spaces.Box(low=np.array([0., -1., 0.]), high=np.array([1., 1., 1.]))

    def __init__(self, path=None):
        if path is None:
            path = os.path.join(brs_data.getDataPath(), "martlet9", "martlet9.urdf")
        URDFRobot.__init__(self, path)

    def reset(self):
        self._initialized_components = False
        return None

    def applyControls(self, bullet_client, controls):
        # Control: [thrust, steer, shoot]
        # thrust in [0,1]
        # steer in [-1, 1]
        # shoot in [0, 1]
        assert self._steer_wheel_joint is not None, "Steer wheel joint not initialized! Was initializeComponents() called?"
        assert self._steer_gun_link is not None, "Steer gun link not initialized! Was initializeComponents() called?"
        assert self._thruster_link is not None, "Thruster link not initialized! Was initializeComponents() called?"
        thrust_force = [0, 0, controls[0] * Martlet9Robot.MAX_THRUST]
        steer_vel = controls[1] * Martlet9Robot.MAX_STEER
        shoot_force = [-controls[2] * Martlet9Robot.MAX_SHOOT, 0, 0]
        p = bullet_client
        p.applyExternalForce(self.uid, self._thruster_link, thrust_force, [0, 0, 0], p.LINK_FRAME)
        p.applyExternalForce(self.uid, self._steer_gun_link, shoot_force, [0, 0, 0], p.LINK_FRAME)
        p.setJointMotorControlArray(self.uid,
                                    [self._steer_wheel_joint],
                                    p.VELOCITY_CONTROL,
                                    targetVelocities=[steer_vel])
        return Martlet9Robot.COSTS @ np.abs(controls)

    def getState(self, bullet_client):
        p = bullet_client
        steer_gun_state = p.getLinkState(self.uid, self._steer_gun_link, computeLinkVelocity=0)
        rocket_pos = p.getBasePositionAndOrientation(self.uid)
        rocket_vel = p.getBaseVelocity(self.uid)
        position = rocket_pos[0]
        orientation = rocket_pos[1]
        velocity = rocket_vel[0]
        ang_vel = rocket_vel[1]
        steer_gun_position = steer_gun_state[0]
        steer_gun_orientation = steer_gun_state[1]
        return np.array([*position,
                         *orientation,
                         *velocity,
                         *ang_vel,
                         *steer_gun_position,
                         *steer_gun_orientation])

    def describeState(state):
        return {
            'position': state[:3],
            'orientation': state[3:7],
            'velocity': state[7:10],
            'ang_vel': state[10:13],
            'steer_position': state[13:16],
            'steer_orientation': state[16:20],
        }

    def addToScene(self, scene, position, orientation):
        URDFRobot.addToScene(self, scene, position, orientation)
        if not self._initialized_components:
            self.initializeComponents(scene.p)
        scene.p.changeDynamics(self.uid, -1, linearDamping=0)

    def initializeComponents(self, bullet_client):
        self.foot1_link = 1
        self.foot2_link = 3
        self.foot3_link = 5
        steer_wheel_base_joint_name = b"base_to_steer1_wheel"
        gun_steer_wheel_joint_name = b"steer1_wheel_to_steer1_gun"
        thruster_casing_joint_name = b"thruster_casing_to_thruster"
        thruster_fire_joint_name = b"thruster_to_fire"
        steer_smoke_joint_name = b"steer1_gun_to_smoke"
        p = bullet_client
        self._initialized_components = True
        num_joints = p.getNumJoints(self.uid)
        for i in range(num_joints):
            joint_info = p.getJointInfo(self.uid, i)
            if joint_info[1] == steer_wheel_base_joint_name:
                self._steer_wheel_joint = i
            elif joint_info[1] == gun_steer_wheel_joint_name:
                self._steer_gun_link = joint_info[-1]
            elif joint_info[1] == thruster_casing_joint_name:
                self._thruster_link = joint_info[-1]
            elif joint_info[1] == thruster_fire_joint_name:
                self.thruster_fire_id = i
            elif joint_info[1] == steer_smoke_joint_name:
                self.steer_smoke_id = i
        if self._steer_wheel_joint is None:
            print("WARNING: Joint named {} not found!".format(steer_wheel_base_joint_name))
        if self._steer_gun_link is None:
            print("WARNING: Joint named {} not found!".format(gun_steer_wheel_joint_name))
        if self._thruster_link is None:
            print("WARNING: Joint named {} not found!".format(thruster_casing_joint_name))
        if self.thruster_fire_id is None:
            print("WARNING: Joint named {} not found!".format(thruster_fire_joint_name))
        if self.steer_smoke_id is None:
            print("WARNING: Joint named {} not found!".format(steer_smoke_joint_name))
