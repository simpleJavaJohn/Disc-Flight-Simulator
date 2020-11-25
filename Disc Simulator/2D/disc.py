import numpy as np

aoa0 = 4.*np.pi/180.

class Disc():
    '''def __init__(self, name, mass, rad, cl0, cla, cd0, cda):
        self.name = name
        self.mass = mass
        self.rad = rad
        self.area = np.pi*rad**2
        self.cl0 = cl0
        self.cla = cla
        self.cd0 = cd0
        self.cda = cda'''

    def __init__(self, name):
        coefs = np.loadtxt("2D\discs.txt", skiprows=1, usecols=(1,2,3))
        models = np.loadtxt("2D\discs.txt", dtype=str, skiprows=1, usecols=(0,))
        for model, coefs in zip(models, coefs):
            if name == model:
                self.name = model
                self.mass = 0.175
                self.rad = 0.13
                self.area = np.pi*self.rad**2
                self.cl0 = coefs[0]
                self.cla = coefs[1]
                self.cd0 = coefs[2]
                self.cda = 0.69

    def coefLift(self, aoa):
        return self.cl0 + self.cla * aoa
    
    def coefDrag(self, aoa):
        return self.cd0 + self.cda * (aoa - aoa0)**2

    def getLinCoef(self, aoa):
        return self.coefLift(aoa), self.coefDrag(aoa)