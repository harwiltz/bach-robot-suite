import pybullet as p
import pybullet_data as pbd
import time

def main():
    client = p.connect(p.GUI)
    p.setAdditionalSearchPath(pbd.getDataPath())
    p.setGravity(0, 0, -10)
    plane_id = p.loadURDF("plane.urdf")
    start_pos = [0, 0, 38]
    start_orientation = p.getQuaternionFromEuler([0, 0, 0])
    martlet_id = p.loadURDF("martlet9.urdf",
                            start_pos, 
                            start_orientation)
    for i in range(1000):
        p.stepSimulation()
        time.sleep(1/240)
    p.disconnect()

if __name__ == "__main__":
    main()
