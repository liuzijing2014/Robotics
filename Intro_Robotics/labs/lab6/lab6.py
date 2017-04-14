"""
Sample Code for Lab5
Use "run.py [--sim] lab3" to execute
"""

from pyCreate2 import create2
from pid_controller import PIDController

import odometry
import math
import matplotlib
# if on the robot, don't use X backend
matplotlib.use('Agg')
import matplotlib.pyplot as plt

class Run:
    def __init__(self, factory):
        """Constructor.

        Args:
            factory (factory.FactoryCreate)
        """
        self.create = factory.create_create()
        self.time = factory.create_time_helper()
        self.controller = PIDController(75, self.time.time(), 40, 15) # initialize PDcontroller
        self.controller_s = PIDController(100, self.time.time(), 50, 17)
        self.odometry = odometry.Odometry() # initialize Odometry

        # desired distance/ goal state
        self.x = 0
        self.y = 0
        self.target_x = 0.5
        self.target_y = 0.7
        self.goal_angle =  0
        self.goal_dis = 0

        # for plot
        self.times = []
        self.position_x = []
        self.position_y = []

        self.wheel_speed = 150

    def run(self):  
        # start moving
        self.create.start()
        self.create.safe()

       # request sensors
        # self.create.start_stream([
        #     create2.Sensor.LeftEncoderCounts,
        #     create2.Sensor.RightEncoderCounts,
        # ])


        start = self.time.time()
        while(True):
            
            distance = math.fabs( math.sqrt((self.target_x - self.x)**2 + (self.target_y - self.y)**2) )
            print(distance)
            print(self.x, self.y)

            time = self.time.time()
            state = self.create.update()
            if state is not None:
                self.odometry.update(state.leftEncoderCounts, state.rightEncoderCounts)

                self.goal_angle = math.atan2( (self.target_y - self.y), (self.target_x - self.x) )

                singal = self.controller.update(self.odometry.theta, self.goal_angle, self.time.time())
                singal_s = self.controller_s.update(distance, 0.01, self.time.time())

                self.create.drive_direct(int(math.fabs(singal_s) + singal),  int(math.fabs(singal_s) + -1 * singal))

                self.x = self.odometry.x
                self.y = self.odometry.y
                self.position_x.append(self.x)
                self.position_y.append(self.y)


            if  distance<= 0.01:
                self.create.drive_direct(0, 0)
                break


        plt.plot(self.position_x, self.position_y)
        plt.plot(self.x, self.y)
        #plt.plot([0, self.goal], [self.times[-1], self.goal], 'k-')
        plt.savefig("plot.png")


