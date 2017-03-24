import lab9_map
import math
import particle_filter as pf
import numpy as np
import odometry
from pyCreate2 import create2


#-------
        # This is an example on how to visualize the pose of our estimated position
        # where our estimate is that the robot is at (x,y,z)=(0.5,0.5,0.1) with heading pi
        # self.virtual_create.set_pose((0.5, 0.5, 0.1), 0)

        # This is an example on how to show particles
        # the format is x,y,z,theta,x,y,z,theta,...
        # data = [0.5, 0.5, 0.1, math.pi/2, 1.5, 1, 0.1, 0]
        # self.virtual_create.set_point_cloud(data)
#-------


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
        self.virtual_create = factory.create_virtual_create()
        self.map = lab9_map.Map("lab9_map.json")

        # specifications
        self.forwardSpeed = 0.5
        self.turnSpeed = 90
        self.motorSpeed = 100
        self.num = 100
        self.sensorNoise = 0.1
        self.forwardNoise = 0.1
        self.turnNoise = 0.1
        self.particles = []
        self.data = []
        self.particleFilter = pf.ParticleFilter(self.sensorNoise, self.forwardNoise, self.turnNoise, self.map)

    def plotVirtualRobots(self):
        self.data = []
        for index in range(self.num):
            particle = self.particles[index]
            self.data += [particle.x, particle.y, 0.0, 0.0]

        self.virtual_create.set_point_cloud(self.data)

    def forward(self, distance):
        old_x = self.odometry.x
        old_y = self.odometry.y
        distance = distance**2
        self.create.drive_direct(self.motorSpeed, self.motorSpeed)

        while True:
            state = self.create.update()
            if state is not None:
                self.odometry.update(state.leftEncoderCounts, state.rightEncoderCounts)
                print("Robot Location is [%f, %f, %f]" % (self.odometry.x, self.odometry.y, math.degrees(self.odometry.theta)))
            currentSqr = (self.odometry.x - old_x)**2 + (self.odometry.y - old_y)**2
            if currentSqr >= distance:
                self.create.drive_direct(0, 0)
                break;

    def turnLeft(self, radians):
        old_theta = self.odometry.theta
        self.create.drive_direct(-self.motorSpeed, self.motorSpeed)

        while True:
            state = self.create.update()
            if state is not None:
                self.odometry.update(state.leftEncoderCounts, state.rightEncoderCounts)
                print("Robot Location is [%f, %f, %f]" % (self.odometry.x, self.odometry.y, math.degrees(self.odometry.theta)))
            if math.fabs(self.odometry.theta - old_theta)  >= radians:
                self.create.drive_direct(0, 0)
                break;

    def run(self):
        # device init
        self.servo.go_to(0)
        self.create.start()
        self.create.safe()

        # necessary for simulation
        self.create.start_stream([
            create2.Sensor.LeftEncoderCounts,
            create2.Sensor.RightEncoderCounts,
        ])

        # create particels
        pLog = math.log(1.0 / self.num)
        for index in range(self.num):
            particle = pf.Particle(np.random.uniform(0, self.map.top_right[0], 1)[0], np.random.uniform(0, self.map.top_right[1], 1)[0], 0.0, pLog)
            self.data += [particle.x, particle.y, 0.0, math.radians(particle.orientation)]
            self.particles.append(particle)

        self.virtual_create.set_point_cloud(self.data)

       
        while True:
            # if a command is issued
            b = self.virtual_create.get_last_button()
            if b == self.virtual_create.Button.MoveForward:
                print("Move forward pressed")

                # move robot and update odometry
                self.forward(self.forwardSpeed)

                # update particle transform
                self.particleFilter.advance(self.particles, self.forwardSpeed, 0)
            
            elif b == self.virtual_create.Button.TurnLeft:
                print("Turn Left pressed!")

                # move robot and update odometry
                self.turnLeft(math.radians(self.turnSpeed))

                # update particle transform
                self.particleFilter.advance(self.particles, 0, -self.turnSpeed)
                
            elif b == self.virtual_create.Button.TurnRight:
                print("Turn Right pressed!")
                self.particleFilter.advance(self.particles, 0, self.turnSpeed)
            elif b == self.virtual_create.Button.Sense:
                print("Sense pressed!")
                self.particles = self.particleFilter.readinSensor(particles, self.sonar.get_distance())
            # do virtualization
            self.plotVirtualRobots()
            self.time.sleep(0.01)
