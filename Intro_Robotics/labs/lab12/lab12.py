from pyCreate2 import create2
import math
import lab12_image
import numpy as np
import matplotlib
# if on the robot, don't use X backend
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import localizer
import drawer

class Run:
    def __init__(self, factory):
        self.create = factory.create_create()
        self.time = factory.create_time_helper()
        self.localizer = localizer.Localizer(factory, self.create, self.time)
        self.drawer = drawer.Drawer(factory, self.create, self.localizer)

        # specif
        self.scale = 0.2

        self.toDraws = []

    def colorComparator(self, left, right):
        return left.color < right.color

    def getNormal(self,p1, p2):
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]

        length = math.sqrt(dy**2 + dx**2)

        norm1 = [-dy/length, dx/length]
        norm2 = [dy/length, -dx/length]

        return norm1, norm2

    def getEndPoints(self, p1, p2):
        if p1[0] == p2[0]:
            if p1[1] < p2[1]:
                end1 = [p1[0], p1[1]]
                end2 = [p2[0], p2[1]]
                return end1, end2
            else:
                end1 = [p2[0], p2[1]]
                end2 = [p1[0], p1[1]]
                return end1, end2
        else:
            if p1[0] < p2[0]:
                end1 = [p1[0], p1[1]]
                end2 = [p2[0], p2[1]]
                return end1, end2
            else:
                end1 = [p2[0], p2[1]]
                end2 = [p1[0], p1[1]]
                return end1, end2

    def run(self):
        self.create.start()
        self.create.safe()

        # request sensors
        self.create.start_stream([
            create2.Sensor.LeftEncoderCounts,
            create2.Sensor.RightEncoderCounts,
        ])

        # read file
        img = lab12_image.VectorImage("lab12_img1.yaml")

        for path in img.paths:
            self.toDraws.append(path)
        for line in img.lines:
            self.toDraws.append(line)

        self.toDraws.sort(self.colorComparator)
        for line in self.toDraws:
            if line.type == "line":
                front, back = self.getEndPoints([line.u[0], line.u[1]], [line.v[0], line.v[1]])
                angle = math.atan2(back[1]-front[1], back[0]-front[0])

                up, down = self.getNormal(front, back)
                print("front=[%f, %f], back=[%f, %f], rFront=[%f, %f], rBack=[%f, %f]" % (front[0], front[1], back[0], back[1], front[0]+up[0], front[1]+up[1], back[0]+up[0], back[1]+up[1]))
                front[0] += up[0] * self.scale
                front[1] += up[1] * self.scale
                back[0] += up[0] * self.scale
                back[1] += up[1] * self.scale

                #plt.plot([line.u[0], line.v[0]], [line.u[1], line.v[1]], line.color)
                self.drawer.go_to_goal_line(front[0], front[1])
                self.drawer.go_to_theta(angle)
                if line.color == "black":
                    self.drawer.go_to_goal_line(back[0], back[1], draw=True, color = (0.0, 0.0, 0.0))
                elif line.color == "green":
                    self.drawer.go_to_goal_line(back[0], back[1], draw=True, color = (0.0, 1.0, 0.0))
                elif line.color == "red":
                    self.drawer.go_to_goal_line(back[0], back[1], draw=True, color = (1.0, 0.0, 0.0))
                elif line.color == "blue":
                    self.drawer.go_to_goal_line(back[0], back[1], draw=True, color = (0.0, 0.0, 1.0))
                
            # end of if
            elif line.type == "curve":
                pass
                # points = []
                # ts = np.linspace(0, 1.0, 10)
                # for i in range(0, path.num_segments()):
                #     for t in ts[:-2]:
                #         s = path.eval(i, t)
                #         points.append(s)
                # print(points)
                # self.drawer.go_to_goal_curve(points)

        self.time.sleep(10)
        self.drawer.go_to_goal_line(-5, -5)
        self.time.sleep(100)
