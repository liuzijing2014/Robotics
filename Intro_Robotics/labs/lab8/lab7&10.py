"""
Sample Code for Lab5
Use "run.py [--sim] lab3" to execute
"""

from pyCreate2 import create2
from pd_controller import PDController
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

        self.servo = factory.create_servo() # initilzie servo
        self.sonar = factory.create_sonar() # initialize sonar

        self.controller = PIDController(340, 10, 60, [-10, 10], [-300, 300], is_angle=True) # initialize PIDcontroller for go to goal
        self.controller_s = PIDController(1000, 0, 50, [0, 0], [-300, 300], is_angle=False) # initialize PIDcontroller for go to goal
        self.controller_w = PDController(1000, 100, -60, 60) # initialize PIDcontroller for following the wall
        self.odometry = odometry.Odometry() # initialize Odometry

        # desired distance/ goal state
        self.x = 0
        self.y = 0
        self.target_x = 0.0
        self.target_y = 0.0
        self.goal_angle =  0

        # desired distance/ goal state for following the wall
        self.goal = 0.0
        self.forward_speed = 170

        # for plot
        self.times = []
        self.position_x = []
        self.position_y = []
        
        # waypoints
        self.waypoints = [
                            # [2.0, 0.0, 0.9],
                            # [3.0, 2.0, 0.9],
                            # [2.5, 2.0, 0.9],
                            # [0.0, 1.5, 0.9],
                            # [0.0, 0.0, 0.5]
                            [8.0 * 0.305, 6.0 * 0.305, 0.55],
                            [7.0 * 0.305, 1.0 * 0.305, 0.55],
                            [1.0 * 0.305, 6.0 * 0.305, 0.55],
                            [0.0 * 0.305, 0.0 * 0.305, 0.55],
                        ]
        self.index = 0

        # 0 -> go to goal, 1 -> follow wall
        self.state = 0
        self.angle = 1
    

    def followingWall(self):
        # turn servo to -90, towards to the wall
        #self.servo.go_to(70)
        #self.time.sleep(.1)
        self.controller_w = PDController(1000, 100, -65, 65)


        self.angle = -70
        self.servo.go_to(self.angle)
        self.create.drive_direct(100, -100)
        start = self.time.time()
        # print(self.x, self.y)
        # print(self.odometry.theta)
        while(self.time.time() - start < 1.2):
            state = self.create.update()
            if state is not None:
                self.odometry.update(state.leftEncoderCounts, state.rightEncoderCounts)
                self.x = self.odometry.x
                self.y = self.odometry.y
                self.position_x.append(self.x)
                self.position_y.append(self.y)           
        self.create.drive_direct(0, 0)
        self.time.sleep(0.1)
        # print(self.x, self.y)
        # print(self.odometry.theta)

        o_dis = math.fabs( math.sqrt((self.target_x - self.x)**2 + (self.target_y - self.y)**2) )
        while(True):
            # update odometry
            state = self.create.update()
            if state is not None:
                self.odometry.update(state.leftEncoderCounts, state.rightEncoderCounts)
                self.x = self.odometry.x
                self.y = self.odometry.y
                self.position_x.append(self.x)
                self.position_y.append(self.y)

            # do wall following
            current = self.sonar.get_distance()
            #print(current)
            self.goal = 0.35 / math.sin(math.radians(math.fabs(self.angle)))
            cur_dis = math.fabs( math.sqrt((self.target_x - self.x)**2 + (self.target_y - self.y)**2) )
            if cur_dis < o_dis and math.fabs(cur_dis - o_dis) > self.waypoints[self.index][2]:
                break
            print(current, self.goal)
            output = self.controller_w.update(current, self.goal, self.time.time())
            print(output)
            self.create.drive_direct(int(self.forward_speed + output), int(self.forward_speed - output))
            self.time.sleep(0.01)

        self.create.drive_direct(0, 0)        

    def goToGoal(self):
        self.controller.init(340, 10, 60, [-10, 10], [-300, 300], is_angle=True) # initialize PIDcontroller for go to goal
        self.controller_s.init(1000, 0, 50, [0, 0], [-300, 300], is_angle=False) # initialize PIDcontroller for go to goal
        
        # turn servo to 0, towards to the wall
        self.servo.go_to(0)
        self.time.sleep(.2)
        self.goal = 0.35

        while(True):         
            # update odometry
            state = self.create.update()
            if state is not None:
                self.odometry.update(state.leftEncoderCounts, state.rightEncoderCounts)
                self.x = self.odometry.x
                self.y = self.odometry.y
                self.position_x.append(self.x)
                self.position_y.append(self.y)

                # do go to goal
                self.goal_angle = math.atan2(self.target_y - self.odometry.y, self.target_x - self.odometry.x)
                output_theta = self.controller.update(self.odometry.theta, self.goal_angle, self.time.time())
                distance = math.sqrt(math .pow(self.target_x - self.odometry.x, 2) + math.pow(self.target_y - self.odometry.y, 2))
                # checking state change
                current = self.sonar.get_distance()
                # if current <= self.goal and distance > 0.5:
                #     break      
                if  distance<= 0.02:
                    self.index += 1
                    if self.index == len(self.waypoints):
                        break
                    else:
                        self.controller.init(340, 10, 60, [-10, 10], [-300, 300], is_angle=True) # initialize PIDcontroller for go to goal
                        self.controller_s.init(1000, 0, 50, [0, 0], [-300, 300], is_angle=False) # initialize PIDcontroller for go to goal
                        self.target_x = self.waypoints[self.index][0]
                        self.target_y = self.waypoints[self.index][1]

                output_distance = self.controller_s.update(0, distance, self.time.time())
                # print(output_distance, output_theta)
                output_distance = max(output_distance, 100)
                self.create.drive_direct(int(output_theta + output_distance), int(-output_theta + output_distance))

        self.create.drive_direct(0, 0)

    def run(self):  
        # start moving
        self.create.start()
        self.create.safe()

        # necessary for simulation
        # self.create.start_stream([
        #     create2.Sensor.LeftEncoderCounts,
        #     create2.Sensor.RightEncoderCounts,
        # ])

        # retriving the target point
        self.target_x = self.waypoints[self.index][0]
        self.target_y = self.waypoints[self.index][1]

        while(self.index < len(self.waypoints)):

            if self.state == 0:
                print("go to the goal")
                self.goToGoal()
                self.state = (self.state + 1) % 2
            else:
                print("follow the wall")
                self.followingWall()
                self.state = (self.state + 1) % 2

        plt.plot(self.position_x, self.position_y)
        plt.plot(self.x, self.y)
        plt.savefig("plot.png")


