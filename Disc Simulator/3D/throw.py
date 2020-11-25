import numpy as np
from numpy.linalg import norm


# Throw Class
#
# Stores position, Euler angles, and linear and angular velocities
# Includes fucntions to calculate helpful values based on above primary values
# All values are stored as vectors (np arrays)
class Throw():
    def __init__(self, name, pos, vel, ang, rot):
        self.name = name
        self.pos = pos
        self.vel = vel
        self.ang = ang
        self.rot = rot

    # Returns AOA which is the difference between angle of velocity and horizontal planes
    def getAoa(self):
        zBHat = self.getBodyHats()[2]
        vDotZBHat = np.dot(self.vel, zBHat)
        vProjected = self.vel - zBHat * vDotZBHat
        aoa = -np.arctan(vDotZBHat / norm(vProjected))
        return aoa

    # Returns unit velocity vectors
    def getNormVel(self):
        return self.vel / norm(self.vel)

    # Returns body hat vectors
    def getBodyHats(self):
        zBHat = self.getRotationMatrix()[2]
        vProjected = self.vel - (zBHat * np.dot(self.vel,zBHat))
        xBHat = vProjected / norm(vProjected)
        yBHat = np.cross(zBHat, xBHat)

        return np.array([xBHat, yBHat, zBHat])

    # Returns rotation matrix
    def getRotationMatrix(self):
        sinPhi = np.sin(self.ang[0])
        cosPhi = np.sin(self.ang[0])
        sinThe = np.sin(self.ang[1])
        cosThe = np.sin(self.ang[1])

        return np.array([[cosThe, sinPhi*sinThe,-sinThe*cosPhi],
                         [0     , cosPhi       , sinPhi       ],
                         [sinThe,-sinPhi*cosThe, cosPhi*cosThe]])