import gym
import pybullet

from pybullet_utils import bullet_client

class BaseURDFBulletEnv(gym.Env):
    def __init__(self, render=False):
        self.scene = None
        self.physicsClientId = -1
        self.ownsPhysicsClient = False
        self.renderable = render
        self.robot = None

    def reset(self):
        if self.physicsClientId < 0:
            self.ownsPhysicsClient = True
            if self.renderable:
                self.p = bullet_client.BulletClient(connection_mode=pybullet.GUI)
            else:
                self.p = bullet_client.BulletClient()
            self.physicsClientId = self.p._client
            self.p.resetSimulation()
        if self.robot is not None:
            return self.robot.reset()
        return None

    def close(self):
        if self.ownsPhysicsClient:
            if self.physicsClientId >= 0:
                self.p.disconnect()
        self.physicsClientId = -1

def parse_collision(collision):
    KEYS = [
        'bodyUniqueIdA',
        'bodyUniqueIdB',
        'linkIndexA',
        'linkIndexB',
        'positionOnA',
        'positionOnB',
        'contactNormalOnB',
        'contactDistance',
        'normalForce',
        'lateralFriction1',
        'lateralFrictionDir1',
        'lateralFriction2',
        'lateralFrictionDir2'
    ]
    return { KEYS[i]: collision[1:][i] for i in range(len(collision) - 1) }

