import lab9_map
import math
import particle_filter as pf
import numpy as np
import odometry
from pyCreate2 import create2
import copy
import random


#-------
        # This is an example on how to visualize the pose of our estimated position
        # where our estimate is that the robot is at (x,y,z)=(0.5,0.5,0.1) with heading pi
        # self.virtual_create.set_pose((0.5, 0.5, 0.1), 0)

        # This is an example on how to show particles
        # the format is x,y,z,theta,x,y,z,theta,...
        # data = [0.5, 0.5, 0.1, math.pi/2, 1.5, 1, 0.1, 0]
        # self.virtual_create.set_point_cloud(data)
#-------

math.inf = float("inf")


class Run:
    def __init__(self, factory):
        """Constructor.

        Args:
            factory (factory.FactoryCreate)
        """
        self.create = factory.create_create()
        self.time = factory.create_time_helper()
        self.servo = factory.create_servo()
        self.sonar = factory.create_sonar()
        self.odometry = odometry.Odometry()
        # Add the IP-address of your computer here if you run on the robot
        self.virtual_create = factory.create_virtual_create("192.168.1.145")
        self.map = lab9_map.Map("lab9_map.json")

        # specifications
        self.radius = 0.16
        self.forwardSpeed = 0.25
        self.turnSpeed = math.pi / 2.0
        self.motorSpeed = 100
        self.num = 500
        self.sensorNoise = 0.2
        self.forwardNoise = 0.01
        self.turnNoise = 0.01
        self.completeThreshold = 0.1
        self.particles = []
        self.data = []
        self.initOrientation = [0.0, math.pi/2, math.pi, -math.pi/2]
        self.particleFilter = pf.ParticleFilter(self.sensorNoise, self.forwardNoise, self.turnNoise, self.map, self.radius)
        self.lotteryPool = []

    def plotVirtualRobots(self):
        self.data = []
        maxProb = -math.inf
        i = 0
        for index in range(self.num):
            particle = self.particles[index]
            self.data += [particle.x, particle.y, 0.0, particle.orientation]
            
            if(particle.pLoc > maxProb):
                i = index
                maxProb = particle.pLoc

        self.virtual_create.set_point_cloud(self.data)
        self.virtual_create.set_pose((self.particles[i].x, self.particles[i].y, 0.0), self.particles[i].orientation)        

    def forward(self, distance):
        old_x = self.odometry.x
        old_y = self.odometry.y
        distance = distance**2
        self.create.drive_direct(self.motorSpeed, self.motorSpeed)

        while True:
            state = self.create.update()
            if state is not None:
                self.odometry.update(state.leftEncoderCounts, state.rightEncoderCounts)
                #print("Robot Location is [%f, %f, %f]" % (self.odometry.x, self.odometry.y, math.degrees(self.odometry.theta)))
            currentSqr = (self.odometry.x - old_x)**2 + (self.odometry.y - old_y)**2
            if currentSqr >= distance:
                self.create.drive_direct(0, 0)
                break;

    def turn(self, radians):
        old_theta = self.odometry.theta
        left = 1
        if radians < 0:
            left = -1
        radians = math.fabs(radians)
        self.create.drive_direct(left * self.motorSpeed, -1 * left * self.motorSpeed)

        while True:
            state = self.create.update()
            if state is not None:
                self.odometry.update(state.leftEncoderCounts, state.rightEncoderCounts)
                #print("Robot Location is [%f, %f, %f]" % (self.odometry.x, self.odometry.y, math.degrees(self.odometry.theta)))
            if math.fabs(self.odometry.theta - old_theta)  >= radians:
                self.create.drive_direct(0, 0)
                break;

    def checkComplete(self):
        
        allX = []
        allY = []
        for particle in self.particles:
            allX.append(particle.x)
            allY.append(particle.y)
        
        stdX = np.std(allX)
        stdY = np.std(allY)

        return stdX < self.completeThreshold and stdY < self.completeThreshold

    def run(self):
        # device init
        self.servo.go_to(0)
        self.create.start()
        self.create.safe()

        # necessary for simulation
        # self.create.start_stream([
        #     create2.Sensor.LeftEncoderCounts,
        #     create2.Sensor.RightEncoderCounts,
        # ])

        # create particels
        pLog = math.log(1.0 / self.num)
        for index in range(self.num):
            particle = pf.Particle(np.random.uniform(self.radius, self.map.top_right[0] - self.radius, 1)[0], np.random.uniform(self.radius, self.map.top_right[1] - self.radius, 1)[0], self.initOrientation[random.randrange(0, len(self.initOrientation))] , pLog)
            particle.estimatedDis = self.map.closest_distance([particle.x, particle.y], particle.orientation)
            self.data += [particle.x, particle.y, 0.0, particle.orientation]
            self.particles.append(particle)

        self.virtual_create.set_point_cloud(self.data)
        self.virtual_create.set_pose((self.particles[0].x, self.particles[0].y, 0.0), self.particles[0].orientation)

       
        counter = 1
        done = False
        while not done:
            # retreive sonar data
            sensorData = self.sonar.get_distance()
            print("forward sensor data %f" % sensorData)
            if(sensorData > (self.forwardSpeed+self.radius)):
                self.lotteryPool.append(3)
            else:
                self.servo.go_to(-90)
                self.time.sleep(2)
                data = self.sonar.get_distance()
                print("left sensor data %f" % data)
                if(data > (self.forwardSpeed+self.radius)):
                    self.lotteryPool.append(2)
                
                self.servo.go_to(90)
                self.time.sleep(4)
                data = self.sonar.get_distance()
                print("right sensor data %f" % data)
                if(data > (self.forwardSpeed+self.radius)):
                    self.lotteryPool.append(1)

                self.servo.go_to(0)
                self.time.sleep(2)
            
            cmd = random.randrange(0, len(self.lotteryPool))
            cmd = self.lotteryPool[cmd]
            print(self.lotteryPool)
            print(cmd)
            self.lotteryPool.clear()

            # 3: move forward
            if cmd == 3:
                print("Move forward pressed")

                # move robot and update odometry
                self.forward(self.forwardSpeed)

                # update particle transform
                self.particleFilter.advanceForward(self.particles, self.forwardSpeed)
            
            # 1: turn left
            elif cmd == 1:
                print("Turn Left pressed!")

                # move robot and update odometry
                self.turn(self.turnSpeed)

                # update particle transform
                self.particleFilter.advanceTurn(self.particles, self.turnSpeed)
                
            # 2: turn right    
            elif cmd == 2:
                print("Turn Right pressed!")
                
                # move robot and update odometry
                self.turn(-self.turnSpeed)

                # update particle transform
                self.particleFilter.advanceTurn(self.particles, -self.turnSpeed)

                
            if (counter < 0):
                # # always do sensing and resampling
                resamples = self.particleFilter.readinSensor(self.particles, self.sonar.get_distance())

                # make sure each instance in the resamples is an instance
                newParticles = list()
                for particle in resamples:
                    newParticles.append(pf.Particle(particle.x, particle.y, particle.orientation, particle.pLoc))
                
                self.particles.clear()
                self.particles = newParticles
                counter = -1
                done = self.checkComplete()
            else:
                counter -= 1

            # do virtualization
            self.plotVirtualRobots()
            self.time.sleep(0.01)

        print("I am done")
        self.time.sleep(100000)
