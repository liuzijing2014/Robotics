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
        self.base_speed = 50
        self.stop_threshold = 0.05
        self.curve_threshold = 0.05
        self.theta_threshold = 0.025

    def go_to_goal_line(self, goal_x, goal_y, draw = False, color = (0.0, 1.0, 0.0)):  
        self.pidTheta.init(300, 5, 50, [-10, 10], [-200, 200], is_angle=True)
        if draw:
            self.penholder.set_color(color[0], color[1], color[2])
            self.penholder.go_to(-0.025)
            self.time.sleep(0.05)

        while True:
            # advance
            self.local.update()
            current_x, current_y, current_theta = self.local.x, self.local.y, self.local.theta
            #print("[%f, %f, %f]" % (current_x, current_y, current_theta))
            distance = math.sqrt((goal_x - current_x)**2 + (goal_y - current_y)**2)
            if distance < self.stop_threshold:
                self.create.drive_direct(0, 0)
                break
            
            goal_theta = math.atan2(goal_y - current_y, goal_x - current_x)
            output_theta = self.pidTheta.update(current_theta, goal_theta, self.time.time())

            self.create.drive_direct(int(output_theta + self.base_speed), int(-output_theta + self.base_speed))

        #lift the pen    
        self.penholder.go_to(0.0)
        self.time.sleep(0.05)

    def go_to_goal_curve(self, points, color=(0.0, 1.0, 0.0)):
        self.pidTheta.init(300, 5, 50, [-10, 10], [-200, 200], is_angle=True)
        self.penholder.set_color(color[0], color[1], color[2])
        self.penholder.go_to(-0.025)
        self.time.sleep(0.05)

        index = 0
        while(index < len(points)):
            goal_x = points[index][0]
            goal_y = points[index][1]

            while True:
                # advance
                self.local.update()
                current_x, current_y, current_theta = self.local.x, self.local.y, self.local.theta
                #print("[%f, %f, %f]" % (current_x, current_y, current_theta))
                distance = math.sqrt((goal_x - current_x)**2 + (goal_y - current_y)**2)
                if distance < self.curve_threshold:
                    self.create.drive_direct(0, 0)
                    break
                
                goal_theta = math.atan2(goal_y - current_y, goal_x - current_x)
                output_theta = self.pidTheta.update(current_theta, goal_theta, self.time.time())

                self.create.drive_direct(int(output_theta + self.base_speed), int(-output_theta + self.base_speed))

            index +=1

        #lift the pen    
        self.penholder.go_to(0.0)
        self.time.sleep(0.05)

    def go_to_theta(self, goal_theta):
        self.pidTheta.init(300, 5, 50, [-10, 10], [-200, 200], is_angle=True)
        while True:
            # advance
            self.local.update()
            current_theta = self.local.theta
            #print("[%f], [%f]" % (current_theta, goal_theta))

            if math.fabs(current_theta - goal_theta) < self.theta_threshold:
                self.create.drive_direct(0, 0)
                break
            
            output_theta = self.pidTheta.update(current_theta, goal_theta, self.time.time())
            self.create.drive_direct(int(output_theta*0.8), int(-output_theta*0.8))

        