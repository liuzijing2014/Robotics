import odometry

class Localizer:
    def __init__(self, factory, create, time):
        self.create = create
        self.time = time
        self.tracker = factory.create_tracker(1, sd_x=0.01, sd_y=0.01, sd_theta=0.01, rate=10)
        
        # specifications
        self.itpWeight = 0.5
        self.initCount = 4

        # init position and orientation
        count = self.initCount
        self.x = 0
        self.y = 0
        self.theta = 0
        while(count > 0):
            r = None
            while(r is None):
                r = self.tracker.query()
                self.time.sleep(0.0)

            self.x += r["position"]["x"]
            self.y += r["position"]["y"]
            self.theta += r["orientation"]["y"]
            count -= 1
    
        self.x /= self.initCount
        self.y /= self.initCount
        self.theta /= self.initCount

        # init odometry
        self.odometry = odometry.Odometry(self.x, self.y, self.theta)

    def interpolate(self, t1, t2, f):
        t3 = []
        for i in range(len(t1)):
            t3.append(t1[i] * (1-f) + t2[i] *f)
        return t3

    def update(self):
        v1 = [0.0, 0.0, 0.0]
        v2 = [0.0, 0.0, 0.0]
        state = self.create.update()
        if state is not None:
            self.odometry.update(state.leftEncoderCounts, state.rightEncoderCounts)
            v1[0] = self.odometry.x
            v1[1] = self.odometry.y
            v1[2] = self.odometry.theta
   
        r = self.tracker.query()
        if r is not None:
            v2[0] = r["position"]["x"]
            v2[1] += r["position"]["y"]
            v2[2] += r["orientation"]["y"]

        if state and r:
            vInterp = self.interpolate(v1, v2, self.itpWeight)
            self.x = vInterp[0]
            self.y = vInterp[1]
            self.theta = vInterp[2]
        elif state:
            self.x = v1[0]
            self.y = v1[1]
            self.theta = v1[2]
        elif r:
            self.x = v2[0]
            self.y = v2[1]
            self.theta = v2[2]
        