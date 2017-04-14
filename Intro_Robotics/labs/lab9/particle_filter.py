import numpy as np
import scipy
import scipy.stats
import math
import lab9_map

math.inf = float("inf")


class Particle:
    def __init__(self, x, y, orientation, proba):
        self.x = x
        self.y = y
        self.orientation = orientation
        self.pLoc = proba

        self.estimatedDis = 0
#----------------------------------------------------

class ParticleFilter:
    def __init__(self, sensorNoise, forwardNoise, turnNoise, map, radius):
        self.sensorNoise = sensorNoise
        self.forwardNoise = forwardNoise
        self.turnNoise = turnNoise
        self.map = map
        self.radius = radius

    def advanceForward(self, particles, forward):
        for particle in particles:
            distanceVariance = np.random.normal(0, self.forwardNoise, 1)[0]
            mforward = forward + distanceVariance

            wallDis = self.map.closest_distance((particle.x, particle.y), particle.orientation)
            mforward = min(mforward, wallDis - self.radius)
        
            particle.x += mforward * math.cos(particle.orientation)
            particle.y += mforward * math.sin(particle.orientation)
            particle.estimatedDis = self.map.closest_distance((particle.x, particle.y), particle.orientation)

    def advanceTurn(self, particles, turn):
        for particle in particles:
            directionVariance = np.random.normal(0, self.turnNoise, 1)[0]
            mturn = turn + directionVariance
            
            particle.orientation = math.fmod(particle.orientation + mturn, 2 * math.pi)
            if particle.orientation > math.pi:
                particle.orientation = -1.0 * (math.pi * 2 - particle.orientation)
            elif particle.orientation < (-1 * math.pi):
                particle.orientation = math.pi * 2 + particle.orientation
            print(math.degrees(particle.orientation))
            particle.estimatedDis = self.map.closest_distance((particle.x, particle.y), particle.orientation)

    def readinSensor(self, particles, data):
        pSum = []
        for index in range(len(particles)):
            particle = particles[index]
            pSensorLoc =  scipy.stats.norm(particle.estimatedDis, self.sensorNoise).pdf(data)
            pSensorLoc = -math.inf if pSensorLoc == 0.0 else math.log(pSensorLoc)
            #print("Estimiated distance is %f sensor data is %f sensor noise is %f pSensorLoc is %f" % (particle.estimatedDis, data, self.sensorNoise, pSensorLoc))
            pSum.append(pSensorLoc + particle.pLoc)
        
        logN = math.log(1) - scipy.misc.logsumexp(pSum)

        pSumNotLog = list()
        for index in range(len(particles)):
            particle = particles[index]
            particle.pLoc = logN + pSum[index]
            pSumNotLog.append(math.e ** particle.pLoc)

        # print(pSumNotLog)
        return self.resample(particles, pSumNotLog)

    def resample(self, particles, pList):
        resamples = np.random.choice(particles, len(particles), p = pList)
        pLog = math.log(1.0 / len(particles))
        for particle in resamples:
            particle.pLoc = pLog
            
        return resamples