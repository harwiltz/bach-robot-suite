import numpy as np
import os

import brs_data

from brs_envs.robot import URDFRobot

class Martlet9Robot(URDFRobot):
    MAX_THRUST = 5e7
    MAX_STEER = 10
    MAX_SHOOT = 3e7
    COSTS = np.array([2., 0., 1.]) / np.array([MAX_THRUST, MAX_STEER, MAX_SHOOT])
    def __init__(self, path=None):
        if path is None:
            path = os.path.join(brs_data.getDataPath(), "martlet9", "martlet9.urdf")
        super(Martlet9Robot, self).__init__(path)
        self._initialized_components = False

    def applyControls(self, bullet_client, controls):
        # Control: [thrust, steer, shoot]
        # thrust in [0,1]
        # steer in [-1, 1]
        # shoot in [0, 1]
        assert self._steer_wheel_joint is not None, "Steer wheel joint not initialized! Was initializeComponents() called?"
        assert self._steer_gun_link is not None, "Steer gun link not initialized! Was initializeComponents() called?"
        assert self._thruster_link is not None, "Thruster link not initialized! Was initializeComponents() called?"
        thrust_force = [0, 0, controls[0] * MAX_THRUST]
        steer_vel = controls[1] * MAX_STEER
        shoot_force = [-controls[2] * MAX_SHOOT, 0, 0]
        p = bullet_client
        p.applyExternalForce(self.uid, self._thruster_link, thrust_force, p.LINK_FRAME)
        p.applyExternalForce(self.uid, self._steer_gun_link, shoot_force, p.LINK_FRAME)
        p.setJointMotorControlArray(self.uid,
                                    [self._steer_wheel_joint],
                                    p.VELOCITY_CONTROL,
                                    targetVelocities=[steer_vel])
        return COSTS @ np.abs(controls)

    def getState(self, bullet_client):
        p = bullet_client
        link_states = p.getLinkStates(self.uid,
                                      [self._steer_gun_link, -1],
                                      computeLinkVelocity=1)
        steer_gun_state = link_states[0]
        rocket_state = link_states[1]
        position = rocket_state[0]
        orientation = rocket_state[1]
        ang_vel = rocket_state[-1]
        steer_gun_position = steer_gun_state[0]
        steer_gun_orientation = steer_gun_state[1]
        return np.array([*position,
                         *orientation,
                         *ang_vel,
                         *steer_gun_position,
                         *steer_gun_orientation])

    def addToScene(self, scene, position, orientation):
        URDFRobot.addToScene(self, scene, position, orientation)
        if not self._initialized_components:
            self.initializeComponents(scene.p)

    def initializeComponents(bullet_client):
        steer_wheel_joint_name = b"base_to_steer1_wheel"
        thruster_casing_joint_name = b"thruster_casing_to_thruster"
        p = bullet_client
        self._initialized_components = True
        num_joints = p.getNumJoints(self.uid)
        for i in range(num_joints):
            joint_info = p.getJointInfo(self.uid, i)
            if joint_info[1] == steer_wheel_joint_name:
                self._steer_wheel_joint = i
                self._steer_gun_link = joint_info[-1]
            elif joint_info[1] == thruster_casing_joint_name
                self._thruster_link = joint_info[-1]
        if self._steer_wheel_joint is None:
            print("WARNING: Joint named {} not found!".format(steer_wheel_joint_name))
        if self._thruster_link is None:
            print("WARNING: Joint named {} not found!".format(thruster_casing_joint_name))
