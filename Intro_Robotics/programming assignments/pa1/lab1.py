"""
Example to move robot forward for 10 seconds
Use "python3 run.py [--sim] example1" to execute
"""

import my_robot
MyRobot = my_robot.MyRobot

class Run:
    def __init__(self, factory):
        """Constructor.

        Args:
            factory (factory.FactoryCreate)
        """
        self.robot = MyRobot(factory)

    def run(self):
        self.robot.base_speed = 100
        self.robot.start()

        self.robot.forward(1000)
        self.robot.turn_left(5)
        self.robot.turn_right(5)

        self.robot.base_speed = 200
        self.robot.backward(1000)

        self.robot.stop()
