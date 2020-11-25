import numpy as np

class Flight():
    def __init__(self, throw):
        self.throw = throw

        self.posDat = np.array([self.throw.pos,])
        self.velDat = np.array([self.throw.vel,])
        self.angDat = np.array([self.throw.ang,])
        self.rotDat = np.array([self.throw.rot,])
        self.tDat = np.array([0.])

    def appendData(self, t):
        self.posDat = np.append(self.posDat, [self.throw.pos,], axis=0)
        self.velDat = np.append(self.velDat, [self.throw.vel,], axis=0)
        self.angDat = np.append(self.angDat, [self.throw.ang,], axis=0)
        self.rotDat = np.append(self.rotDat, [self.throw.rot,], axis=0)
        self.tDat = np.append(self.tDat, t)

    def getDist(self):
        return self.posDat[-1]