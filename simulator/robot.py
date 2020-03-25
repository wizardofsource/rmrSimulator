import struct
import math
import sys

from helpertypes import XYPair, LeftRightPair
from sensors import Gyro, IRCSensor, Lidar
from geometry2 import llintersectpoint, dist, mmToM, mToCm, cmToMm, pointToLineDistance, cmToM, mToMm
from debug import printd, pause

def idf(x):
    return x

robotDefaultCtor = {"r" : (idf, [0.035]), "d" : (idf, [0.23]), "pos" : (XYPair, [1,1]), "speeds" : (LeftRightPair, [0,0]), "fi" : (idf, [math.pi/2]), "w" : (idf, [0]), "gyro" : (Gyro, []), "leftIRC" : (IRCSensor, []), "rightIRC" : (IRCSensor, []), "lidar" : (Lidar, [0, 0, 2*math.pi*7.9]), "requestQueue" : (idf, [[]])}
#2*math.pi*7.9

robotAttribCtors = {"pos": lambda x: (XYPair, x), "fi" : lambda x: (idf, [x])}

def fillFromIni(ctor, inidict):
    for k,v in inidict['robot'].items():
        if k not in robotAttribCtors:
            print("Wrong syntax in inifile in section [Robot] at key %s" % (k, ))
            sys.exit(1)
        robotDefaultCtor[k] = robotAttribCtors[k](eval(inidict['robot'][k]));
    return robotDefaultCtor

class Robot:
    def __init__(self, ctordict): # r=0.035, d=0.23, pos=XYPair(1,1), speeds=LeftRightPair(0,0), fi=0, w=0, gyro = Gyro(), leftIRC = IRCSensor(), rightIRC = IRCSensor(), lidar = Lidar(0, 0, 2*math.pi*7.9)): # w should be 2*pi*7.9
        self.resetCtor = ctordict
        self.construct(self.resetCtor)

    def construct(self, ctordict):
        for k,v in ctordict.items():
            ctor,args = v
            if args:
                setattr(self, k, ctor(*args))
            else:
                setattr(self, k, ctor())

    def isCollision(self, se):
        bl = se.areamap.boundarylines
        x = self.pos.x
        y = self.pos.y

        for l in bl:
            d = pointToLineDistance([x, y], l)
            if self.d > d:
                return True

        for o in se.areamap.objects:
            for l in o.lines:
                d = pointToLineDistance([x, y], l)
                if self.d > d:
                    printd("Collision d=%s" %(d, ))
                    return True
        return False


    def enqueueRequest(self, s):
        self.requestQueue.append(s)

    def serveRequests(self, deltatime, locks, se):
        while self.requestQueue:
            r = self.requestQueue[0]
            if r == "reset":
                self.reset(deltatime, locks, se) 
            self.requestQueue = self.requestQueue[1:]

    def reset(self, deltatime, locks, se):
        locks['robotposlock'].acquire()
        locks['sensorlock'].acquire()
        self.construct(self.resetCtor)
        locks['sensorlock'].release()
        locks['robotposlock'].release()

    def updateCords(self, deltatime, locks, se):
        locks['robotposlock'].acquire()
        jointspeed = (self.speeds.right + self.speeds.left) / 2
        #printd("Jointed speed: %s" % (jointspeed,))
        rollbackx = self.pos.x;
        rollbacky = self.pos.y;
        self.pos.x += math.cos(self.fi)*jointspeed*deltatime
        self.pos.y += math.sin(self.fi)*jointspeed*deltatime
        # printd("FI : %s " %(self.fi))
        # Comment this if it's too slow
        if self.isCollision(se):
            printd("COLLISION")
            self.pos.x = rollbackx;
            self.pos.y = rollbacky;
        printd("ROBOT POS: %s %s" % (self.pos.x , self.pos.y))
        self.w = (self.speeds.right - self.speeds.left)/self.d
        self.fi += self.w*deltatime
        while (self.fi > math.pi*2):
            self.fi = self.fi - 2*math.pi
        while (self.fi < 0):
            self.fi = self.fi + 2*math.pi
        locks['robotposlock'].release()

    def updateSensors(self, deltatime, locks, se):
        self.lidar.update(deltatime, self, se.areamap, locks)
        self.gyro.update(self.w, self.fi)

        locks['sensorlock'].acquire()
        self.leftIRC.update(deltatime*(self.speeds.left))
        self.rightIRC.update(deltatime*(self.speeds.right))
        locks['sensorlock'].release()

    def update(self, deltatime, locks, se):
        self.serveRequests(deltatime, locks, se)
        self.updateCords(deltatime, locks, se)
        self.updateSensors(deltatime, locks, se)

    def execute(self, cmd, cmdstr):
        if cmd[:9] == [0xaa, 0x55, 0x0A, 0x0c, 0x02, 0xf0, 0x00, 0x01, 0x04]:
            hdr1, hdr2, speed, radius = struct.unpack("<Qbhh", cmdstr[:13])

            speedinm = mmToM(speed)
            printd("SPEEDINM : %s" % (speedinm, ))
            if radius == 1:
                printd("only rotate %s" % (speedinm, ))
                self.onlyRotate(speedinm)
            elif radius == 0:
                printd("only translate")
                self.onlyTranslate(speedinm)
            elif radius > 1 or radius < -1: 
                radiusinm = mmToM(radius)
                printd("arc %s %s" % (speedinm, radius))
                self.arc(speedinm, radiusinm)
            else:
                printd("Malformed command, stopping...")
                self.onlyTranslate(0)

    def onlyTranslate(self, newspeed):
        self.speeds.left = newspeed
        self.speeds.right = newspeed
    
    def onlyRotate(self, rotspeed):
        self.speeds.left = -rotspeed/2
        self.speeds.right = rotspeed/2

    def arc(self, speedinm, radiusinm):
        # if radiusinm > 1mm
        # r_stred = radius + b/2
        # v_tan = speedinm*radius/(radius + b/2) 
        #   => vleft + vright = 2*v_tan
        # wziad = v_tan/r_stredu 
        #   => vright - vleft =  d*v_tan/r_stredu
        # ===> 2*vright = 2*v_tan + d*v_tan/r_stredu
        # ===> 2*vleft = 2*v_tan - d*v_tan/r_stredu
        # ===> vright = v_tan(1 + (d/2)/r_stredu)
        # ===> vleft = v_tan(1 - (d/2)/r_stredu)
        # if radiusinm < -1mm
        # r_stred = radius - b/2
        # v_tan = speedinm*radius/(radius - b/2) 
        #   => vleft + vright = 2*v_tan
        # wziad = v_tan/r_stredu 
        #   => vright - vleft =  d*v_tan/r_stredu
        # ===> vright = v_tan(1 + (d/2)/r_stredu)
        # ===> vleft = v_tan(1 - (d/2)/r_stredu)

        # if radiusinm > 0.001:
        #     r_stredu = radiusinm + self.d/2
        # else:
        #     r_stredu = radiusinm - self.d/2
        r_stredu = radiusinm
            
        v_tan = speedinm*radiusinm/r_stredu
        self.speeds.right = v_tan*(1 + (self.d/2)/r_stredu)
        self.speeds.left = v_tan*(1 - (self.d/2)/r_stredu)
