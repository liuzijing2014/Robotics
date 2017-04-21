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

def cmp_to_key(mycmp):
    'Convert a cmp= function into a key= function'
    class K:
        def __init__(self, obj, *args):
            self.obj = obj
        def __lt__(self, other):
            return mycmp(self.obj, other.obj) < 0
        def __gt__(self, other):
            return mycmp(self.obj, other.obj) > 0
        def __eq__(self, other):
            return mycmp(self.obj, other.obj) == 0
        def __le__(self, other):
            return mycmp(self.obj, other.obj) <= 0
        def __ge__(self, other):
            return mycmp(self.obj, other.obj) >= 0
        def __ne__(self, other):
            return mycmp(self.obj, other.obj) != 0
    return K

class Run:
    def __init__(self, factory):
        self.create = factory.create_create()
        self.time = factory.create_time_helper()
        self.localizer = localizer.Localizer(factory, self.create, self.time)
        self.drawer = drawer.Drawer(factory, self.create, self.localizer)

        # specif
        self.scale = 0.2
        self.rrtThreshold = 1.0

        self.toDraws = []
        self.map = lab_map.Map("lab12_obs1.png")
        self.rrt = rrt.RRT(self.map, 50, 50)

    def convertCoordinates(self, point):
        cur_x = point[0] * 100.0
        cur_y = self.map.img.height - point[1] * 100.0
        return [cur_x, cur_y]
    
    def convertCoordinatesToSim(self, point):
        cur_x = point[0] / 100.0
        cur_y = (self.map.img.height - point[1]) / 100.0
        return [cur_x, cur_y]

    def findPath(self, x, y):
        curPos = self.convertCoordinates([self.localizer.x, self.localizer.y])
        goalPos = self.convertCoordinates([x,y])

        self.rrt.build((curPos[0], curPos[1]), 2000, 10)
        x_goal = self.rrt.nearest_neighbor((goalPos[0], goalPos[1]))
        path = self.rrt.shortest_path(x_goal)

        # for v in self.rrt.T:
        #     for u in v.neighbors:
        #         self.map.draw_line((v.state[0], v.state[1]), (u.state[0], u.state[1]), (0,0,0))
        # for idx in range(0, len(path)-1):
        #     self.map.draw_line((path[idx].state[0], path[idx].state[1]), (path[idx+1].state[0], path[idx+1].state[1]), (0,255,0))

        # self.map.save("lab11_rrt.png")
        
        pathMap = []
        for p in path:
            trueP = self.convertCoordinatesToSim([p.state[0], p.state[1]])
            pathMap.append(trueP)

        return pathMap

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

    def getCurvePoints(self, points):
        leftToRight = []
        rightToLeft = []
        for i in range(len(points)):
            if i > 0 and i < (len(points) - 1):
                up1, down1 = self.getNormal(points[i-1], points[i])
                up2, down2 = self.getNormal(points[i], points[i+1])
                up = [up1[0]*0.5+up2[0]*0.5, up1[1]*0.5+up2[1]*0.5]
                down = [down1[0]*0.5+down2[0]*0.5, down1[1]*0.5+down2[1]*0.5]
                leftToRight.append([points[i][0]+up[0]*self.scale, points[i][1]+up[1]*self.scale])
                rightToLeft.append([points[i][0]+down[0]*self.scale, points[i][1]+down[1]*self.scale])
        
        for p in leftToRight:
            pMap = self.convertCoordinates(p)
            if self.map.has_obstacle(pMap[0], pMap[1]):
                print("curve left to right collided [%f, %f]" % (p[0], p[1]))
                rightToLeft.reverse()
                return rightToLeft
        return leftToRight
    
    def checkCollision(self, front, back):
        distance = math.sqrt((back[0] - front[0])**2 + (back[1] - front[1])**2)
        step = 1/(distance*(1/self.scale))
        t = 0.0
        p = [0,0]
        result = False
        while t <= 1.0:
            p[0] = front[0]*(1-t) + back[0]*t
            p[1] = front[1]*(1-t) + back[1]*t
            pM = self.convertCoordinates(p)
            if self.map.has_obstacle(pM[0], pM[1]):
                print("[%f, %f] collide [%f, %f]" % (p[0], p[1], pM[0], pM[1]))
                return True
            else:
                t += step
        
        return False

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
        
        if self.checkCollision(front, back):
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

        print(self.map.has_obstacle(89.523455, 109.093047))

        # read file
        img = lab12_image.VectorImage("lab12_img1.yaml")
        # print("x", self.map.width)
        # print("y", self.map.height)
        for path in img.paths:
            self.toDraws.append(path)
        for line in img.lines:
            self.toDraws.append(line)

        self.toDraws.sort(key=cmp_to_key(self.colorComparator))
        for line in self.toDraws:

            current_x = self.drawer.local.x
            current_y = self.drawer.local.y

            if line.type == "line":                
                front, back = self.getLineEndPoints([line.u[0], line.u[1]], [line.v[0], line.v[1]])
                angle = math.atan2(back[1]-front[1], back[0]-front[0])
                print("front=[%f, %f], back=[%f, %f], rFront=[%f, %f], rBack=[%f, %f]" % (line.u[0], line.u[1], line.v[0], line.v[1], front[0], front[1], back[0], back[1]))

                if self.checkCollision([self.localizer.x, self.localizer.y], front):
                    print("RTT")
                    toStart = self.findPath(front[0], front[1])
                    self.drawer.go_to_goal_curve(toStart)
                    self.drawer.go_to_goal_line(front[0], front[1])
                else:
                    print("No RTT")
                    toAngle = math.atan2(front[1]-self.localizer.y, front[0]-self.localizer.x)
                    self.drawer.go_to_theta(toAngle)
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
                points = []
                ts = np.linspace(0, 1.0, 10)
                for i in range(0, path.num_segments()):
                    for t in ts[:-2]:
                        s = path.eval(i, t)
                        points.append(s)
                validPoints = self.getCurvePoints(points)
                initAngle = math.atan2(validPoints[1][1]-validPoints[0][1], validPoints[1][0]-validPoints[0][0])
                #distance = math.sqrt((validPoints[0][0] - self.localizer.x)**2 + (validPoints[0][1] - self.localizer.y)**2)

                if self.checkCollision([self.localizer.x, self.localizer.y], validPoints[0]):
                    toStart = self.findPath(validPoints[0][0], validPoints[0][1])
                    self.drawer.go_to_goal_curve(toStart)
                    self.drawer.go_to_goal_line(validPoints[0][0], validPoints[0][1])
                else:
                    toAngle = math.atan2(validPoints[0][1]-self.localizer.y, validPoints[0][0]-self.localizer.x)
                    self.drawer.go_to_theta(toAngle)
                    self.drawer.go_to_goal_line(validPoints[0][0], validPoints[0][1])
                
                self.drawer.go_to_theta(initAngle)
                validPoints.pop(0)
                if line.color == "black":
                    self.drawer.go_to_goal_curve(validPoints, draw=True, color = (0.0, 0.0, 0.0))
                elif line.color == "green":
                    self.drawer.go_to_goal_curve(validPoints, draw=True, color = (0.0, 1.0, 0.0))
                elif line.color == "red":
                    self.drawer.go_to_goal_curve(validPoints, draw=True, color = (1.0, 0.0, 0.0))
                elif line.color == "blue":
                    self.drawer.go_to_goal_curve(validPoints, draw=True, color = (0.0, 0.0, 1.0))

        self.time.sleep(10)
        self.drawer.go_to_goal_line(0, 0)
        self.time.sleep(100)
