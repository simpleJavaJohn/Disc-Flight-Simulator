import numpy as np

aoa0 = 4.*np.pi/180.


# Disc Class
#
# Stores the coefficients and physical attributes as described by Hummel 2003
# Also calculates total coefficients based on AOA
class Disc():
    def __init__(self, name, mass, rad, cl0, cla, cd0, cda, cm0, cma, cmq, crr, crp, cnr):
        self.name = name
        self.mass = mass
        self.rad = rad
        self.area = np.pi*rad**2
        self.cl0 = cl0
        self.cla = cla
        self.cd0 = cd0
        self.cda = cda
        self.cm0 = cm0
        self.cma = cma
        self.cmq = cmq
        self.crr = crr
        self.crp = crp
        self.cnr = cnr

    def coefLift(self, aoa):
        return self.cl0 + self.cla * aoa
    
    def coefDrag(self, aoa):
        return self.cd0 + self.cda * (aoa - aoa0)**2

    # Returns both linear coefficients unpacked
    def getLinCoef(self, aoa):
        return self.coefLift(aoa), self.coefDrag(aoa)

    def coefR(self, dPhi, dGam):
        return self.crr*dGam + self.crp*dPhi

    def coefM(self, aoa, dThe):
        return self.cm0 + self.cma*aoa + self.cmq*dThe

    def coefN(self, dGam):
        return self.cnr * dGam

    # Returns all angular coefficients in array
    def getAngCoef(self, aoa, dPhi, dThe, dGam):
        return np.array([self.coefR(dPhi, dGam), self.coefM(aoa, dThe), self.coefN(dGam)])