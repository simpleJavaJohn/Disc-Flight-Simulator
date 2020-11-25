import numpy as np
from numpy.linalg import norm

class Throw():
    def __init__(self, vel, aor, aoa):
        aor *= np.pi/180.
        aoa *= np.pi/180
        vx = np.cos(aor)*vel
        vz = np.sin(aor)*vel

        pos = np.array([0., 1.])
        vel = np.array([vx, vz])
        alp = aor + aoa

        self.pos = pos
        self.vel = vel
        self.alp = alp

    def getAoa(self):
        angVel = np.arctan(self.vel[1]/self.vel[0])
        aoa = self.alp - angVel
        #print(f'aoa={aoa*180/np.pi}')
        return(aoa)

    def getNormVel(self):
        return self.vel / norm(self.vel)

    def getSpeed(self):
        return norm(self.vel)