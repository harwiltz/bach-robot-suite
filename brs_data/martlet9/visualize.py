import pybullet as p
import pybullet_data as pbd
import time

def main():
    client = p.connect(p.GUI)
    p.setAdditionalSearchPath(pbd.getDataPath())
    p.setGravity(0, 0, -9.81)
    plane_id = p.loadURDF("plane.urdf")
    start_pos = [0, 0, 70]
    start_orientation = p.getQuaternionFromEuler([0, 0.30, 0])
#    start_orientation = p.getQuaternionFromEuler([0, 0.00, 0])
    martlet_id = p.loadURDF("martlet9.urdf",
                            start_pos, 
                            start_orientation)
    theta = 0
    d = 20
    while True:
        p.resetDebugVisualizerCamera(cameraDistance=d,
                                     cameraPitch=0,
                                     cameraYaw=theta,
                                     cameraTargetPosition=[2,1,5])
        p.stepSimulation()
        time.sleep(1/240)
        theta += 0.1
        if d < 70:
            d += 0.01
    p.disconnect()

if __name__ == "__main__":
    main()
