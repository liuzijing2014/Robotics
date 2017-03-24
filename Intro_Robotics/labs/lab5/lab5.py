"""
Sample Code for Lab5
Use "run.py [--sim] lab3" to execute
"""

from pyCreate2 import create2
from pd_controller import PDController
import odometry
import math

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
        self.controller = PDController(200, self.time.time(), 50) # initialize Pcontroller

        # desired distance/ goal state
        self.goal = 0.45 / math.sin(math.radians(70))
        self.forward_speed = 75

    def run(self):
        # turn servo to -90, towards to the wall
        self.servo.go_to(70)
        self.time.sleep(1)

        # start moving
        self.create.start()
        self.create.safe()

        # request sensors
        # self.create.start_stream([
        #     create2.Sensor.LeftEncoderCounts,
        #     create2.Sensor.RightEncoderCounts,
        # ])

        limit = self.time.time()
        while(True):
            current = self.sonar.get_distance()
            left, right = self.controller.update(current, self.goal, self.time.time())
            self.create.drive_direct(int(self.forward_speed + left), int(self.forward_speed + right))
            self.time.sleep(0.15)
            if(self.time.time() - limit >= 65):
                break


