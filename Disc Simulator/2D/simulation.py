import numpy as np
from throw import *
from disc import *
from flight import *
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

airDensity  = 1.2 #kg/m^3
g           = 9.81 #m/s/s

class Simulation():
    def __init__(self, disc, throw, flight, dt):
        self.disc = disc
        self.throw = throw
        self.flight = flight
        self.dt = dt

    def simulate(self, show=True):

        t = 0.

        while self.throw.pos[1] >= 0.:
            self.move()
            t += self.dt
            self.flight.appendData(t)

        if show: self.show2D()

    def getEnd(self):
        self.simulate(show=False)
        return self.flight.posDat[-1][0]

    def move(self):
        self.throw.vel += self.calcAcc() * self.dt

        self.throw.pos += self.throw.vel * self.dt

    def calcAcc(self):
        return self.calcForce() / self.disc.mass

    def calcForce(self):
        aoa = self.throw.getAoa()
        speed = self.throw.getSpeed()
        normVel = self.throw.getNormVel()

        coefLift, coefDrag = self.disc.getLinCoef(aoa)
        force = 0.5 * airDensity * speed**2 * self.disc.area

        forceLift = coefLift * force * np.cross(np.array([[0,1],[-1,0]]), normVel)
        forceDrag = coefDrag * force * -normVel
        forceGrav = self.disc.mass * g * np.array([0., -1.])

        sumForces = forceLift + forceDrag + forceGrav
        return sumForces

    def show2D(self, autoScale=True):
        x, z = self.flight.posDat[:,0], self.flight.posDat[:,1]        
        plt.plot(x, z)
        plt.show(block=False)
        #aoa = self.flight.aoaDat
        #plt.plot(x,aoa)
        #plt.show(block=False)

def test():
    models = np.loadtxt("2D\discs.txt", dtype=str, skiprows=1, usecols=(0,))
    for model in models:
        disc = Disc(model)
        aors = []
        aoas = []
        dists = []

        for aor in [0, 5, 10, 15, 20]:
            for aoa in [0, 2, 4, 6, 8]:
                throw = Throw(20., aor, aoa)
                flight = Flight(throw)
                simulation = Simulation(disc, throw, flight, 0.01)
                dist = simulation.getEnd()

                aors.append(aor)
                aoas.append(aoa)
                dists.append(dist)
        
        fig = plt.figure()
        ax = fig.gca(projection='3d')
        ax.plot(aors, aoas, dists)
        ax.set_xlabel('aors')
        ax.set_ylabel('aoas')
        ax.set_zlabel('dists')
        ax.set_title(f'{disc.name}')
        plt.show(block=False)
        break
    plt.show()

def testAor():
    models = np.loadtxt("2D\discs.txt", dtype=str, skiprows=1, usecols=(0,))
    for model in models:
        disc = Disc(model)
        aors = []
        dists = []

        for aor in np.linspace(0., 60., 25):
            throw = Throw(20., aor, 1.)
            flight = Flight(throw)
            simulation = Simulation(disc, throw, flight, 0.01)
            dist = simulation.getEnd()

            aors.append(aor)
            dists.append(dist)
        
        plt.plot(aors, dists, label=disc.name)
    plt.title('Angle of release (deg) vs max distance (m)')
    plt.xlabel('aor (deg)')
    plt.ylabel('dist (m)')
    plt.legend()
    plt.show()

def testAoa():
    models = np.loadtxt("2D\discs.txt", dtype=str, skiprows=1, usecols=(0,))
    for model in models:
        disc = Disc(model)
        aoas = []
        dists = []

        for aoa in np.linspace(0., 20., 50):
            throw = Throw(20., 15., aoa)
            flight = Flight(throw)
            simulation = Simulation(disc, throw, flight, 0.01)
            dist = simulation.getEnd()

            aoas.append(aoa)
            dists.append(dist)
        
        plt.plot(aoas, dists, label=disc.name)
    plt.title('Angle of attack (deg) vs max distance (m)')
    plt.xlabel('aoa (deg)')
    plt.ylabel('dist (m)')
    plt.legend()
    plt.show()

if __name__ == "__main__":
    '''models = np.loadtxt("2D\discs.txt", dtype=str, skiprows=1, usecols=(0,))
    for model in models:
        throw = Throw(20., 0., 0.)

        disc = Disc(model)

        flight = Flight(throw)

        simulation = Simulation(disc, throw, flight, 0.01)

        simulation.simulate()
    plt.show()'''
    testAor()
    #test()
    testAoa()
    