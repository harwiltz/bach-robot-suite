import pybullet
import gym

class Scene:
    def __init__(self, bullet_client=None, gravity=9.8, timestep=1/240, sticky=1):
        self.p = bullet_client
        self.timestep = timestep
        self.sticky = sticky
        self._gravity = gravity
        self.reset()

    def step(self):
        self.p.stepSimulation()

    def reset(self):
        self.p.setGravity(0, 0, -self._gravity)
        self.p.setPhysicsEngineParameter(fixedTimeStep=self.timestep * self.sticky,
                                         numSubSteps=self.sticky)
