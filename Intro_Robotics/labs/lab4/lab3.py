"""
Sample Code for Lab3
Use "run.py [--sim] lab3" to execute
"""

from pyCreate2 import create2
import math

class Run:
    def __init__(self, factory):
        """Constructor.

        Args:
            factory (factory.FactoryCreate)
        """
        self.create = factory.create_create()
        self.time = factory.create_time_helper()
        self.x = 0
        self.y = 0
        self.a = 0
        self.rc = 0
        self.lc = 0

    def run(self):
            
        self.create.start()
        self.create.safe()

        # request sensors
        self.create.start_stream([
            create2.Sensor.LeftEncoderCounts,
            create2.Sensor.RightEncoderCounts,
        ])

        while True:
            state = self.create.update()
            if state is not None:
                print(state.__dict__)

        # self.create.start()
        # self.create.safe()

        # # request sensors
        # self.create.start_stream([
        #     create2.Sensor.LeftEncoderCounts,
        #     create2.Sensor.RightEncoderCounts,
        # ])

        # state = self.create.update()
        # if state is not None:
        #     self.rc = state.__dict__['rightEncoderCounts']
        #     self.lc = state.__dict__['leftEncoderCounts']

        # self.create.drive_direct(1000, 500)
        # limit = self.time.time()
        # while(True):
        #     state = self.create.update()
        #     self.printinfo(state)
        #     if(self.time.time() - limit >= 20):
        #         break

        # self.create.stop()

    def printinfo(self, state):
        state = self.create.update()
        if state is not None:
            disrc = 0
            dislc = 0
            newrc = state.__dict__['rightEncoderCounts']
            newlc = state.__dict__['leftEncoderCounts']
            print(str(self.rc) + ',' + str(self.lc))
            print(str(newrc) + ',' + str(newlc))

            if(math.fabs(newrc - self.rc) >=  32767):
                self.rc = newrc

            if(math.fabs(newlc - self.lc) >=  32767):
                self.lc = newlc

            disrc = newrc - self.rc
            self.rc = newrc
            
            dislc = newlc - self.lc
            self.lc = newlc

            r = disrc / 508.8 * math.pi * 72
            l = dislc / 508.8 * math.pi * 72
            self.a += (r-l) / 235
            d = (r+l) / 2

            self.x += math.cos(self.a) * d
            self.y += math.sin(self.a) * d

            print('[' + str(self.x) + ',' + str(self.y) +',' + str(math.degrees(self.a)) + ']')