"""
Robot module provides movement control for the robot.
"""

class MyRobot:
    """ Robot controller class """

    def __init__(self, factory):
        """Constructor.

        Args:
            factory (factory.FactoryCreate)
        """
        self.create = factory.create_create()
        self.time = factory.create_time_helper()
        self.base_speed = 10

    def forward(self, distance):
        """
        Forward movement

        Arguments:
            distance: how far the robot is moving forward (meters)
        """
        self.create.drive_direct(self.base_speed, self.base_speed)
        self.time.sleep(distance / self.base_speed)

    def backward(self, distance):
        """
        Backward movement

        Arguments:
            distance: how far the robot is moving backward (meters)
        """
        self.create.drive_direct(-self.base_speed, -self.base_speed)
        self.time.sleep(distance / self.base_speed)

    def turn_left(self, duration):
        """
        Left turn movement

        Arguments:
            duration: how long the robot is turning left (second)
        """
        self.create.drive_direct(-self.base_speed, self.base_speed)
        self.time.sleep(duration)

    def turn_right(self, duration):
        """
        Right turn movement

        Arguments:
            duration: how long the robot is turning right (second)
        """
        self.create.drive_direct(self.base_speed, -self.base_speed)
        self.time.sleep(duration)

    def stop(self):
        """
        Stop robot movement
        """
        self.create.stop()

    def start(self):
        """
        Start robot movement
        """
        self.create.start()
        self.create.safe()

        