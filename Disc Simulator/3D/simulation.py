import numpy as np
from throw import *
from disc import *
from flight import *
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Constant values
airDensity  = 1.2 #kg/m^3
g           = 9.81 #m/s/s
Izz         = 0.002352 
Ixx=Iyy=Ixy = 0.001219 

# Simulator Class
#
# Holds a throw and a disc object as variables
# Stores flight data in flight object
# Calculates all forces and accelerations
# and simulates the flight of a disc until it reaches the ground
class Simulation():
    def __init__(self, disc, throw, flight, dt):
        self.disc = disc
        self.throw = throw
        self.flight = flight
        self.dt = dt

    # Initial simulation call
    # Calculates acceleration and moves disc until hits the ground
    # Stores values in flight object
    def simulate(self, show=True):

        t = 0.

        while self.throw.pos[2] >= 0.:
            self.move()
            t += self.dt
            self.flight.appendData(t)

        if show: self.show3D()

    # Calculates accelerations and updates velocity and position by the Euler Method
    def move(self):
        self.throw.vel += self.calcLinAcc() * self.dt
        self.throw.rot += self.calcAngAcc() * self.dt

        self.throw.pos += self.throw.vel * self.dt
        self.throw.ang += self.throw.rot * self.dt

    # Returns linear acceleration
    def calcLinAcc(self):
        return self.calcLinForce() / self.disc.mass

    # Returns angular acceleration
    def calcAngAcc(self):
        angForce = self.calcAngForce()
        sinThe, cosThe = np.sin(self.throw.ang[1]), np.cos(self.throw.ang[1])
        dPhi, dThe, dGam = self.throw.rot[0], self.throw.rot[1], self.throw.rot[2]

        phiAcc = (angForce[0] + 2*Ixy*dPhi*dThe*sinThe - Izz*dThe*(dPhi*sinThe+dGam))*cosThe/Ixy
        theAcc = (angForce[1] + Izz*dPhi*cosThe*(dPhi*sinThe+dGam) - Ixy*dPhi**2*cosThe*sinThe)/Ixy
        gamAcc = (angForce[2] - Izz*phiAcc*sinThe + dThe*dPhi*cosThe)/Izz

        angAcc = np.array([phiAcc, theAcc, gamAcc])
        return angAcc

    # Calculates sum of linear forces (lift, drag, gravity)
    def calcLinForce(self):
        aoa = self.throw.getAoa()
        force = self.calcForceMatrix()
        coefLift, coefDrag = self.disc.getLinCoef(aoa)
        normVel = self.throw.getNormVel()
        bodyHats = self.throw.getBodyHats()

        forceLift = coefLift * force * np.cross(normVel, bodyHats[1])
        forceDrag = coefDrag * force * -normVel
        forceGrav = self.disc.mass * g * np.array([0., 0., -1.])

        sumForces = forceLift + forceDrag + forceGrav
        return sumForces

    # Calculates base linear force according to Bernoulli's equation
    def calcForceMatrix(self):
        v2 = np.dot(self.throw.vel, self.throw.vel)
        force = 0.5 * airDensity * self.disc.area * v2
        return force

    # Calculates angular forces (torques)
    def calcAngForce(self, prnt=False):
        aoa = self.throw.getAoa()
        angCoef = self.disc.getAngCoef(aoa, self.throw.rot[0], self.throw.rot[1], self.throw.rot[2])
        torque = self.calcTorqueMatrix()
        sumTorques = angCoef * torque
        if prnt: print("Tor={0:4.2f}, X={1}, Y={2}, Z= {3}, sum={4}".format(torque, torqueX, torqueY, torqueZ, sumTorques))
        return sumTorques

    # Calculates base angular force (torque)
    def calcTorqueMatrix(self):
        v2 = np.dot(self.throw.vel, self.throw.vel)
        torque = 0.5 * airDensity * self.disc.area * self.disc.rad*2 * v2
        return torque

    # Dispalys trajectory of a flight
    def show3D(self, autoScale=True, dim="3D"):
        x, y, z = self.flight.posDat[:,0], self.flight.posDat[:,1], self.flight.posDat[:,2]
        if dim == "3D":
            fig = plt.figure()
            ax = fig.gca(projection='3d')
            if not autoScale:
                axMin = min(min(x), min(y), min(z))
                axMax = max(max(x), max(y), max(z))
                ax.set_xlim(axMin, axMax)
                ax.set_ylim(axMin, axMax)
                ax.set_zlim(axMin, axMax)
            ax.plot(x, y, z)
            ax.set_xlabel('x-axis')
            ax.set_ylabel('y-axis')
            ax.set_zlabel('z-axis')
            ax.set_title(f'{self.disc.name} thrown {self.throw.name}')
        else:
            plt.plot(x, z, label=self.disc.name)
            plt.show(block=False)

# Load disc from disc.txt based on name (str)
# Defaults to hummel's coefs
def loadDisc(name):
    allCoefs = np.loadtxt("3D\discs.txt", skiprows=1, usecols=(1,2,3,4,5))
    models = np.loadtxt("3D\discs.txt", dtype=str, skiprows=1, usecols=(0,))
    for model, coefs in zip(models, allCoefs):
        if name == model:
            name = model
            mass = 0.175
            rad = 0.13
            area = np.pi*rad**2
            cl0 = coefs[0]
            cla = coefs[1]
            cd0 = coefs[2]
            cda = 0.69
            cm0 = coefs[3]
            cma = coefs[4]
            cmq = -5.0e-3 #-5.0e-2
            crr = 1.4e-2 #0.43
            crp = -5.5e-3 #-5.5e-2
            cnr = -7.1e-6 #-7.1e-5
            #print(f'{name}, l0={cl0}, la={cla}, d0={cd0}, da={cda}')
            return Disc(name, mass, rad, cl0, cla, cd0, cda, cm0, cma, cmq, crr, crp, cnr)
    print("Name didn't match, loading hummel")
    return loadDisc("hummel")

# Load throw from throws.txt based on name (str) unless specific roll angle is given (phi)
def loadThrow(name, phi = 400):
    if phi == 400:
        initVals = np.loadtxt("3D\\throws.txt", skiprows=1, usecols=(1,2,3,4))
        throwNames = np.loadtxt("3D\\throws.txt", dtype=str, skiprows=1, usecols=(0,))
        for throwName, vals in zip(throwNames, initVals):
            if name == throwName:
                name = throwName
                v = vals[0]
                a = vals[1] *np.pi/180
                vx = np.cos(a)*v
                vz = np.sin(a)*v
                pos = np.array([0., 0., 1.])
                vel = np.array([vx, 0., vz])
                ang = np.array([vals[2]*np.pi/180, a*1.1, 0.])
                rot = np.array([0., 0., 0.])
                throw = Throw(name, pos, vel, ang, rot)
                return throw
        print("Name didn't match, loading flat")
        return loadThrow("flat")
    else:
        name = (f'phi={phi}')
        v = 20.
        a = 10. *np.pi/180
        vx = np.cos(a)*v
        vz = np.sin(a)*v
        pos = np.array([0., 0., 1.])
        vel = np.array([vx, 0., vz])
        ang = np.array([phi*np.pi/180, a*1.05, 0.])
        rot = np.array([0., 0., 0.])
        throw = Throw(name, pos, vel, ang, rot)
        return throw

# An example test to display relationship between roll and range
def testPhivsDist():
    N = 60
    dists = np.array([])
    phis = np.linspace(-100, 60, N)
    for phi in phis:
        throw = loadThrow("", phi=phi)
        flight = Flight(throw)
        disc = loadDisc("hummel")
        simulation = Simulation(disc, throw, flight, 0.01)
        simulation.simulate(show=False)
        dist = simulation.flight.getDist()[0]
        dists = np.append(dists, dist)
    plt.plot(phis, dists)
    plt.title('Phi (deg) vs maximum range (m)')
    plt.xlabel('Phi (deg)')
    plt.ylabel('dist (m)')

# Main method ran when python file is ran
# Currently displays a 3D flight and then runs an example test
if __name__ == "__main__":
    '''throw = loadThrow("anheizer")
    flight = Flight(throw)
    for name in ["aviar", "buzz", "roc", "flick", "storm", "wraith", "quarter", "hummel", "pc", "sc", "key"]:
        disc = loadDisc(name)
    
        simulation = Simulation(disc, throw, flight, 0.001)
        simulation.simulate()'''

    '''disc = loadDisc("hummel")
    for name in ["flat", "heizer", "veryHeizer", "anheizer", "veryAnheizer"]:
        throw = loadThrow(name)
        flight = Flight(throw)
        simulation = Simulation(disc, throw, flight, 0.01)
        simulation.simulate()'''
    
    throw = loadThrow("anheizer")
    flight = Flight(throw)
    disc = loadDisc("aviar")
    simulation = Simulation(disc, throw, flight, 0.01)
    simulation.simulate()
    plt.show()

    testPhivsDist()
    plt.legend()
    plt.show()
    