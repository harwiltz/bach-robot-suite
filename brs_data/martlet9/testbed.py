import numpy as np
import pybullet as p
import pybullet_data as pbd
import time

def simulate(steps):
    for i in range(steps):
        p.stepSimulation()
        time.sleep(1/240)

def simulate_with(steps, fn):
    for i in range(steps):
        fn()
        p.stepSimulation()
        time.sleep(1/240)

client = p.connect(p.GUI)
p.setAdditionalSearchPath(pbd.getDataPath())
p.setGravity(0, 0, -10)
plane_id = p.loadURDF("plane.urdf", [0, 0, -4])
water_orientation = p.getQuaternionFromEuler([np.pi/2, 0, 0])
water_id = p.loadURDF("../water.urdf", [0, 0, -1], water_orientation)
p.changeVisualShape(water_id, -1, rgbaColor=[0.2, 0.2, 0.6, 0.7])
pad_id = p.loadURDF("landingpad.urdf", [0, 0, -0.5])
start_pos = [0, 0, 50]
start_orientation = p.getQuaternionFromEuler([0, 0, 0])
martlet_id = p.loadURDF("martlet9.urdf",
                        start_pos,
                        start_orientation)
simulate(1000)
