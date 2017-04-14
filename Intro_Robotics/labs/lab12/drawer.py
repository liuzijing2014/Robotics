"""
Example to use the pen holder
Use "python3 run.py [--sim] lab12_penholder_test" to execute
"""
import math
import pid_controller
import localizer


class Drawer:
    def __init__(self, factory, create, localizer):
        self.create = create
        self.time = factory.create_time_helper()
        self.penholder = factory.create_pen_holder()
        self.pidTheta = pid_controller.PIDController(300, 5, 50, [-10, 10], [-200, 200], is_angle=True)
        self.pidDistance = pid_controller.PIDController(1000, 0, 50, [0, 0], [-200, 200], is_angle=False)
        self.local = localizer
        
        # specifications
        self.base_speed = 100
        self.stop_threshold = 0.05
        self.theta_threshold = 0.01

    def go_to_goal_line(self, goal_x, goal_y, draw = False, color = (0.0, 1.0, 0.0)):  

        if draw:
            self.penholder.set_color(color)
            self.penholder.go_to(-0.025)
            self.time.sleep(0.05)

        while True:
            # advance
            self.local.update()
            current_x, current_y, current_theta = self.local.x, self.local.y, self.local.theta
            print("[%f, %f, %f]" % (current_x, current_y, current_theta))

            if math.fabs(current_x - goal_x) < self.stop_threshold and math.fabs(current_y - goal_y) < self.stop_threshold:
                self.create.drive_direct(0, 0)
                break
            
            goal_theta = math.atan2(goal_y - current_y, goal_x - current_x)
            #theta = math.atan2(math.sin(current_theta), math.cos(current_theta))
            output_theta = self.pidTheta.update(current_theta, goal_theta, self.time.time())

            # distance = math.sqrt(math.pow(goal_x - current_x, 2) + math.pow(goal_y - current_y, 2))
            # output_distance = self.pidDistance.update(0, distance, self.time.time())
            self.create.drive_direct(int(output_theta + self.base_speed), int(-output_theta + self.base_speed))

        #lift the pen    
        self.penholder.go_to(0.0)
        self.time.sleep(0.05)

    def go_to_theta(self, goal_theta):
        while True:
            # advance
            self.local.update()
            current_theta = self.local.theta
            print("[%f]" % (current_theta))

            if math.fabs(current_theta - goal_theta) < self.theta_threshold:
                self.create.drive_direct(0, 0)
                break
            
            output_theta = self.pidTheta.update(current_theta, goal_theta, self.time.time())
            self.create.drive_direct(int(output_theta), int(-output_theta))