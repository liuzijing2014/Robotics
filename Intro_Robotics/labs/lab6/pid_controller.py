"""
Robot P-Contoller for following the wall
"""

import math

class PIDController:
    def __init__(self, rotation_costant, time, damping_constant, integral_constant):
        """
        Constructor

        Args:
            rotation_constant: kp
            damping_constant: kd
            integral_constant: ki
        """
        self.turning_constant = rotation_costant
        self.damping_constant = damping_constant
        self.integral_constant = integral_constant
        self.pre_error = 0
        self.pre_time = time
        self.sum_error = 0

    def update(self, current, goal, time):
        """
        Calculate the speeds for each wheel depends on current and goal state

        Returns:
            tuple(left wheel dlta, right wheel dtla)
        """
        error = goal - current
        delt_error = error - self.pre_error
        delt_time = time - self.pre_time
        self.pre_error = error
        self.pre_time = time
        self.sum_error += delt_time * (self.pre_error + error) / 2 

        result = self.turning_constant * error + (delt_error / delt_time) * self.damping_constant
        iterm = self.integral_constant * self.sum_error

        limit = 70
        if result < -limit:
            result = -limit
        elif result > limit:
            result = limit

        ilimit = 50
        if iterm < -ilimit:
            iterm = -ilimit
        elif iterm > ilimit:
            iterm = ilimit

        

        return result + iterm
    
