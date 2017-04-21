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
import rrt
import lab_map

class Run:
    def __init__(self, factory):
        self.create = factory.create_create()
        self.time = factory.create_time_helper()
        self.localizer = localizer.Localizer(factory, self.create, self.time)
        self.drawer = drawer.Drawer(factory, self.create, self.localizer)

        # specif
        self.scale = 0.2

        self.toDraws = []
        self.map = lab_map.Map("lab12_obs1.png")
        self.rrt = rrt.RRT(self.map)

    def convertCoordinates(self, point):
        cur_x = point[0] * 100.0
        cur_y = (1.2 - point[1]) * 100.0
        return [cur_x, cur_y]

    def findPath(self, x, y):
        curPos = self.convertCoordinates([self.localizer.x, self.localizer.y])
        goalPos = self.convertCoordinates([x,y])

        self.rrt.build((curPos[0], curPos[1]), 2000, 5)
        x_goal = self.rrt.nearest_neighbor((goalPos[0], goalPos[1]))
        path = self.rrt.shortest_path(x_goal)
        return path

    def colorComparator(self, left, right):
        if left.color < right.color:
            return -1
        elif left.color > right.color:
            return 1
        else:
            return 0

    def getNormal(self,p1, p2):
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]

        length = math.sqrt(dy**2 + dx**2)

        norm1 = [-dy/length, dx/length]
        norm2 = [dy/length, -dx/length]

        return norm1, norm2

    def getLineEndPoints(self, p1, p2):
        # the end point that is relatively left or bottom
        leftBot = []
        # the end point that is relateively right or top
        rightTop = []

        if p1[0] == p2[0]:
            if p1[1] < p2[1]:
                leftBot = [p1[0], p1[1]]
                rightTop = [p2[0], p2[1]]
            else:
                leftBot = [p2[0], p2[1]]
                rightTop = [p1[0], p1[1]]
        else:
            if p1[0] < p2[0]:
                leftBot = [p1[0], p1[1]]
                rightTop = [p2[0], p2[1]]
            else:
                leftBot = [p2[0], p2[1]]
                rightTop = [p1[0], p1[1]]

        up, down = self.getNormal(leftBot, rightTop)
        front = [leftBot[0]+up[0] * self.scale, leftBot[1]+up[1] * self.scale]
        back = [rightTop[0]+up[0] * self.scale, rightTop[1]+up[1] * self.scale]

        # check collision
        distance = math.sqrt((back[0] - front[0])**2 + (back[1] - front[1])**2)
        step = 1/(distance*(1/self.scale))
        t = 0.0
        p = []
        result = False
        while t <= 1.0:
            p[0] = front[0]*(1-t) + back[0]*t
            p[1] = front[1]*(1-t) + back[1]*t
            p = self.convertCoordinates(p)
            if self.map.has_obstacle(p[0], p[1]):
                result = True
                break
            else:
                t += step
        
        if result:
            front = [rightTop[0]+down[0] * self.scale, rightTop[1]+down[1] * self.scale]
            back = [leftBot[0]+down[0] * self.scale, leftBot[1]+down[1] * self.scale]

        return front, back

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
        # print("x", self.map.width)
        # print("y", self.map.height)
        for path in img.paths:
            self.toDraws.append(path)
        for line in img.lines:
            self.toDraws.append(line)

        self.toDraws.sort(self.colorComparator)
        for line in self.toDraws:

            current_x = self.drawer.local.x
            current_y = self.drawer.local.y

            if line.type == "line":
                # front, back = self.getEndPoints([line.u[0], line.u[1]], [line.v[0], line.v[1]])
                # angle = math.atan2(back[1]-front[1], back[0]-front[0])

                # up, down = self.getNormal(front, back)
                # print("front=[%f, %f], back=[%f, %f], rFront=[%f, %f], rBack=[%f, %f]" % (front[0], front[1], back[0], back[1], front[0]+up[0], front[1]+up[1], back[0]+up[0], back[1]+up[1]))
                # front[0] += up[0] * self.scale
                # front[1] += up[1] * self.scale
                # back[0] += up[0] * self.scale
                # back[1] += up[1] * self.scale
                
                front, back = self.getEndPoints([line.u[0], line.u[1]], [line.v[0], line.v[1]])

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
