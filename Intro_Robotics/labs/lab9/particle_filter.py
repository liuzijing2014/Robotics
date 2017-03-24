import numpy as np
import scipy
import math
import lab9_map

class Particle:
    def __init__(self, x, y, orientation, proba):
        self.x = x
        self.y = y
        self.orientation = orientation
        self.pLoc = proba

        self.estimatedDis = 0
#----------------------------------------------------

class ParticleFilter:
    def __init__(self, sensorNoise, forwardNoise, turnNoise, map):
        self.sensorNoise = sensorNoise
        self.forwardNoise = forwardNoise
        self.turnNoise = turnNoise
        self.map = map

    def advance(self, particles, forward, turn):
        for particle in particles:
            distanceVariance = np.random.normal(0, self.forwardNoise, 1)[0]
            directionVariance = np.random.normal(0, self.turnNoise, 1)[0]

            forward += distanceVariance
            turn += directionVariance

            particle.x += forward * math.cos(math.radians(turn))
            particle.y += forward * math.sin(math.radians(turn))
            particle.estimatedDis = self.map.closest_distance((particle.x, particle.y), particle.orientation)

    def readinSensor(self, particles, data):
        pSum = []
        for index in range(len(particles)):
            particle = particles[index]
            pSensorLoc =  scipy.stats.norm(particle.estimatedDis, self.sensorNoise).pdf(data)
            pSensorLoc = math.log(pSensorLoc)
            pSum.append(pSensorLoc + particle.pLoc)
        
        logN = math.log(1) - scipy.misc.logsumexp(pSum)

        for index in range(len(particles)):
            particle = particles[index]
            particle.pLoc = logN + pSum[index]
            pSum[index] = particle.pLoc

        self.resample(particles, pSum)

    def resample(self, particles, pList):
        resamples = np.random.choice(particles, len(particles), p = pList)
        pLog = math.log(1.0 / len(particles))
        for particle in particles:
            particle.pLoc = pLog
            
        return resamples