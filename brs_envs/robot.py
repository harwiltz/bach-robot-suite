class Robot:
    def addToScene(self, position, orientation):
        pass

    def applyControls(self, controls):
        pass

    def getState(self, scene):
        pass

class URDFRobot(Robot):
    def __init__(self, path):
        super(URDFRobot, self).__init__()
        self._path = path

    def addToScene(self, scene, position, orientation):
        self.uid = scene.p.loadURDF(self._path, position, orientation)
        return self.uid
