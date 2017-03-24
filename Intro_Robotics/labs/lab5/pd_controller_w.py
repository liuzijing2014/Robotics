"""
Robot P-Contoller for following the wall
"""

import math

class PDController:
    def __init__(self, rotation_costant, time, damping_constant):
        """
        Constructor

        Args:
            forward_constant: the default forward speed
            rotation_costant: the default of turning speed (relative)
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

        left = -1 * self.turning_constant * error + -1 * (delt_error / delt_time) * self.damping_constant
        right = self.turning_constant * error + (delt_error / delt_time) * self.damping_constant
        # print(left)
        # print(right)

        limit = 50
        if left < -limit:
            left = -limit
        elif left > limit:
            left = limit
        
        if right < -limit:
            right = -limit
        elif right > limit:
            right = limit


        # we are too close to the wall
        # need to turn right
        # if error > 0:
        return left, right
        # we are too far to the wall
        # need to turn left
        #elif error < 0:
        #   return self.turning_constant * error, -1 * self.turning_constant * error
        # we are right at the desired position
        # no need to turn, just move forward
        #else:
        #    return 0, 0
    
