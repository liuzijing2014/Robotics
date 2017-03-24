"""
Robot P-Contoller for following the wall
"""

import math

class PDController:
    def __init__(self, rotation_costant, time, damping_constant):
        """
        Constructor

        Args:
            rotation_constant: kp
            damping_constant: kd
        """
        self.turning_constant = rotation_costant
        self.damping_constant = damping_constant
        self.pre_error = 0
        self.pre_time = time

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

        #left = -1 * self.turning_constant * error + -1 * (delt_error / delt_time) * self.damping_constant
        result = self.turning_constant * error + (delt_error / delt_time) * self.damping_constant

        limit = 25
        if result < -limit:
            result = -limit
        elif result > limit:
            result = limit
        

        return result
    
