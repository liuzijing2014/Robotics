from pyCreate2 import create2
import math
import numpy as np
import matplotlib
# if on the robot, don't use X backend
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import localizer
import drawer

class Run:
    def __init__(self, factory):
        print("1")
        self.create = factory.create_create()
        print("2")
        self.time = factory.create_time_helper()
        print("3")
        self.localizer = localizer.Localizer(factory, self.create, self.time)
        print("4")
        self.drawer = drawer.Drawer(factory, self.create, self.localizer)

    def run(self):
        self.create.start()
        self.create.safe()

        # request sensors
        self.create.start_stream([
            create2.Sensor.LeftEncoderCounts,
            create2.Sensor.RightEncoderCounts,
        ])

        goal_x = 1
        goal_y = -1
        base_speed = 100

        self.drawer.go_to_goal_line(goal_x, goal_y)
        self.drawer.go_to_theta(0)

        self.time.sleep(10)
