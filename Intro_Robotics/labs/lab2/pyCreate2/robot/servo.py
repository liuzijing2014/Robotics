"""
Module to control a Parallax Servo.
This is a template only and needs to be finished in Lab2
"""
from pyCreate2.robot.pwm import Pwm

class Servo:

    def __init__(self, number):
        """Constructor.

        Args:
            number (integer): PWM number where the servo is connected to.
        """
        self.m_Pwm = Pwm(number)
        self.m_Pwm.set_frequency(50)
        self.m_Pwm.enable()
        self.lower_bound = (0.75/20) * 100
        self.upper_bound = (2.25/20) * 100

    def go_to(self, angle):
        """Go to specified target angle.

        Args:
            angle (float): -90 - 90 degrees. 0 means facing forward. Negative numbers turn to the left.
        """
        angle_percentage = (angle + 90.0) / 180.0
        pulse_width = (1 - angle_percentage) * self.lower_bound + angle_percentage * self.upper_bound;
        self.m_Pwm.set_duty_cycle(pulse_width)


