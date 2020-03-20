import numpy as np
import os
import pybullet_data

import brs_data

from brs_envs.scene import Scene

class RocketLandingScene(Scene):
    def reset(self):
        Scene.reset(self)
        plane_path = os.path.join(pybullet_data.getDataPath(), "plane.urdf")
        self.plane = self.p.loadURDF(plane_path, [0, 0, -4])
        water_path = os.path.join(brs_data.getDataPath(), "water.urdf")
        water_ori = self.p.getQuaternionFromEuler([np.pi/2, 0, 0])
        self.water = self.p.loadURDF(water_path, [0, 0, -1], water_ori)
        self.p.changeVisualShape(self.water, -1, rgbaColor=[0.2, 0.2, 0.6, 0.7])
        pad_path = os.path.join(brs_data.getDataPath(), "martlet9", "landingpad.urdf")
        self.pad = self.p.loadURDF(pad_path, [0, 0, -0.5])
