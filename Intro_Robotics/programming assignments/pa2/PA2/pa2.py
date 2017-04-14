"""
Code for PA2
Use "run.py [--sim] pa2" to execute
"""

import math


class Run:
    def __init__(self, factory):
        """Constructor.

        Args:
            factory (factory.FactorySimulation)
        """
        self.arm = factory.create_kuka_lbr4p()
        self.time = factory.create_time_helper()
        
        # specifications
        self.l1 = 0.3105
        self.l2 = 0.7105 - 0.3105
        self.l3 = 1.1005 - 0.7105

        self.joint4 = (-119, 119)
        self.joint2 = (-105, 105)

        self.paintSpeed = 0.03

        # task specified
        self.FKTests =[(45, -90), (90, -70), (20, -100), (45, -10), (-20, 45), (-110, 105)]
        self.IKTests =[(-0.0079, 0.8690), (-0.5359, 0.6745), (0.2476, 0.7536), (-0.5098, 0.9106), (-0.0279, 1.0398), (0.4112, 0.5608)]
        self.rectangle = [(-0.3, 1.0), (0.3, 1.0), (0.3, 0.9), (-0.3, 0.9), (-0.3, 1.0)]
        self.tri1 = [(-0.3, 1.0), (-0.1, 0.75), (-0.5, 0.75)]
        self.tri2 = [(0.3, 1.0), (0.1, 0.75), (0.5, 0.75)]


    def getPosition(self, joint1, joint2):
        """
        Estimiated position based on joints angle

        Args:
            joint1: angle for joint 2
            joint2: angle for joint 4
        Return:
            x, y coordinates
        """
        joint1 *= -1.0
        joint2 *= -1.0
        x = math.sin(math.radians(joint1)) * self.l2 + math.sin(math.radians(joint1 + joint2)) * self.l3
        y = self.l1 + math.cos(math.radians(joint1)) * self.l2 + math.cos(math.radians(joint1 + joint2)) * self.l3
        return x, y

    def getAngle(self, x, y):
        """
        Estimated joint angles given x, y coordinates

        Args:
            x, y coordinates
        Return:
            angles for joint 2 and joint 4
        """
        y -= self.l1
        t1 = (self.l2**2 + self.l3**2 - x**2 - y**2) / (2 * self.l2 * self.l3)
        t2 = (x**2 + y**2 + self.l2**2 - self.l3**2) / (2 * self.l2 * math.sqrt(x**2 + y**2))

        alpha = math.acos( t1 )
        beta = math.asin( t2 )

        theta11 = math.atan2(y, x) + beta
        theta12 = math.atan2(y, x) - beta
        theta21 = math.pi + alpha
        theta22 = math.pi - alpha

        if theta12 >= math.radians(self.joint2[0]) and theta12 <= math.radians(self.joint2[1]):
            if theta22 >= math.radians(self.joint4[0]) and theta22 <= math.radians(self.joint4[1]):
                return theta12, -1.0 * theta22
        if theta11 >= math.radians(self.joint2[0]) and theta11 <= math.radians(self.joint2[1]):
            if theta21 >= math.radians(self.joint4[0]) and theta21 <= math.radians(self.joint4[1]):
                return theta11, -1.0 * theta21

        return 0.0, 0.0

    def run(self):
        """
        Program entry
        """
        # turn end-effector towards the wall
        self.arm.go_to(5, math.pi / 2.0)
        self.arm.go_to(4, math.pi / -2.0)

        print("Using joint 2 (range %d to %d degree)" % (self.joint2[0], self.joint2[1]))
        print("Using joint 4 (range %d to %d degree)" % (self.joint4[0], self.joint4[1]))

        # FK
        for entry in self.FKTests:
            self.arm.go_to(1, math.radians(entry[0]))
            self.arm.go_to(3, math.radians(entry[1]))
            x, y = self.getPosition(entry[0], entry[1])
            print("Go to %f,%f deg, FK: [%f, 0.06, %f]" % (entry[0], entry[1], x, y))
            self.time.sleep(3)

        # IK
        for entryIK in self.IKTests:
            angle1, angle2 = self.getAngle(entryIK[0], entryIK[1])
            self.arm.go_to(1, angle1)
            self.arm.go_to(3, angle2)
            print("Go to [%f,%f], IK: [%f deg, %f deg]" % (entryIK[0], entryIK[1], math.degrees(angle1), math.degrees(angle2)))
            self.time.sleep(3)

        # Draw rectangle first attempt
        print("First Attempt to draw the rectangle")
        self.arm.set_color(0.0, 0.0, 1.0)
        for point in self.rectangle:
            angle1, angle2 = self.getAngle(point[0], point[1])
            self.arm.go_to(1, angle1)
            self.arm.go_to(3, angle2)
            self.time.sleep(0.5)
            self.arm.enable_painting()
        self.arm.disable_painting()

        # second attempt
        print("Second Attempt to draw the rectangle")
        # Draw top side
        self.time.sleep(5)
        self.arm.set_color(0.0, 0.0, 1.0)
        x = self.rectangle[0][0]
        target = self.rectangle[1][0]
        y = self.rectangle[0][1]
        while(x <= target):
            angle1, angle2 = self.getAngle(x, y)
            self.arm.go_to(1, angle1)
            self.arm.go_to(3, angle2)
            self.time.sleep(0.3)
            self.arm.enable_painting()
            x += self.paintSpeed

        # Draw right side
        self.arm.set_color(0.0, 1.0, 0.0)
        y = self.rectangle[1][1]
        target = self.rectangle[2][1]
        x = self.rectangle[1][0]
        while(y >= target):
            angle1, angle2 = self.getAngle(x, y)
            self.arm.go_to(1, angle1)
            self.arm.go_to(3, angle2)
            self.time.sleep(0.3)
            y -= self.paintSpeed

        # Draw bottom side
        self.arm.set_color(1.0, 0.0, 0.0)
        x = self.rectangle[2][0]
        target = self.rectangle[3][0]
        y = self.rectangle[2][1]
        while(x >= target):
            angle1, angle2 = self.getAngle(x, y)
            self.arm.go_to(1, angle1)
            self.arm.go_to(3, angle2)
            self.time.sleep(0.3)
            x -= self.paintSpeed

        # Draw left side
        self.arm.set_color(1.0, 1.0, 0.0)
        y = self.rectangle[3][1]
        target = self.rectangle[0][1]
        x = self.rectangle[3][0]
        while(y <= target):
            angle1, angle2 = self.getAngle(x, y)
            self.arm.go_to(1, angle1)
            self.arm.go_to(3, angle2)
            self.time.sleep(0.3)
            y += self.paintSpeed
        
        self.arm.disable_painting()

        #custom drawing
        print("Custom drawing")
        self.time.sleep(5)
        self.arm.set_color(0.0, 0.0, 1.0)
        t = 0.0
        while t <= 1.0:
            x = self.tri1[0][0] * (1.0 - t) + self.tri1[1][0] * t
            y = self.tri1[0][1] * (1.0 - t) + self.tri1[1][1] * t
            angle1, angle2 = self.getAngle(x, y)
            self.arm.go_to(1, angle1)
            self.arm.go_to(3, angle2)
            self.time.sleep(0.3)
            self.arm.enable_painting()
            t += 0.05
        
        t = 0.0
        while t <= 1.0:
            x = self.tri1[1][0] * (1.0 - t) + self.tri1[2][0] * t
            y = self.tri1[1][1] * (1.0 - t) + self.tri1[2][1] * t
            angle1, angle2 = self.getAngle(x, y)
            self.arm.go_to(1, angle1)
            self.arm.go_to(3, angle2)
            self.time.sleep(0.3)
            self.arm.enable_painting()
            t += 0.05
        
        t = 0.0
        while t <= 1.0:
            x = self.tri1[2][0] * (1.0 - t) + self.tri1[0][0] * t
            y = self.tri1[2][1] * (1.0 - t) + self.tri1[0][1] * t
            angle1, angle2 = self.getAngle(x, y)
            self.arm.go_to(1, angle1)
            self.arm.go_to(3, angle2)
            self.time.sleep(0.3)
            self.arm.enable_painting()
            t += 0.05

        self.arm.disable_painting()

        self.time.sleep(2)
        self.arm.set_color(0.0, 1.0, 0.0)
        t = 0.0
        while t <= 1.0:
            x = self.tri2[0][0] * (1.0 - t) + self.tri2[1][0] * t
            y = self.tri2[0][1] * (1.0 - t) + self.tri2[1][1] * t
            angle1, angle2 = self.getAngle(x, y)
            self.arm.go_to(1, angle1)
            self.arm.go_to(3, angle2)
            self.time.sleep(0.3)
            self.arm.enable_painting()
            t += 0.05
        
        t = 0.0
        while t <= 1.0:
            x = self.tri2[1][0] * (1.0 - t) + self.tri2[2][0] * t
            y = self.tri2[1][1] * (1.0 - t) + self.tri2[2][1] * t
            angle1, angle2 = self.getAngle(x, y)
            self.arm.go_to(1, angle1)
            self.arm.go_to(3, angle2)
            self.time.sleep(0.3)
            self.arm.enable_painting()
            t += 0.05
        
        t = 0.0
        while t <= 1.0:
            x = self.tri2[2][0] * (1.0 - t) + self.tri2[0][0] * t
            y = self.tri2[2][1] * (1.0 - t) + self.tri2[0][1] * t
            angle1, angle2 = self.getAngle(x, y)
            self.arm.go_to(1, angle1)
            self.arm.go_to(3, angle2)
            self.time.sleep(0.3)
            self.arm.enable_painting()
            t += 0.05

        self.arm.disable_painting()
        self.time.sleep(3)

            