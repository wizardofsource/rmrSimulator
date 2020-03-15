import math

from geometry2 import llintersectpoint, dist, mmToM, mToCm, cmToMm, pointToLineDistance, cmToM, mToMm

class Gyro:
    def __init__(self, w=0, fi=0):
        self.fi = fi
        self.w = w

    def update(self, w, fi, noise=0):
        self.w = w
        self.fi = fi

    def getAngle(self):
        val = int(self.fi*180/math.pi)
        return val

    def getAngleRate(self):
        return 0

class IRCSensor:
    def __init__(self, increments=0):
        self.increments = increments
        self.tickToMeter = 0.000085292090497737556558; 
    def update(self, meterspassed):
        self.increments += meterspassed/self.tickToMeter
    def getIncrements(self):
        return int(self.increments) & 0xffff

class Lidar:
    def __init__(self, fi=0, dist=0, w=0, lastintersectionsqueuesize=100):
        self.fi = fi
        self.dist = dist
        self.w = w # in rad/s
        self.q = []
        for i in range(1, lastintersectionsqueuesize):
            self.q.append([0, 0])

    def update(self, deltatime, robot, areamap, locks):
        locks['robotposlock'].acquire()
        rx = robot.pos.x
        ry = robot.pos.y
        rf = robot.fi
        locks['robotposlock'].release()

        clines = [] # collision lines
        for l in areamap.boundarylines:
            clines.append(l)

        for obj in areamap.objects:
            for l in obj.lines:
                clines.append(l)

        scanp = None # Collision point of scanner ray with collidable line
        mind = None # Ray can intersect many lines, closests one is the correct one

        for l in clines:
            ip = llintersectpoint(rx, ry, rx + 1000*math.cos(rf + self.fi), ry + 1000*math.sin(rf + self.fi), l[0][0], l[0][1], l[1][0], l[1][1])
            if ip is not None:
                #printd("found ip: %s" % ([rx, ry, rx + 1000*math.cos(self.fi), ry - 1000*math.sin(self.fi)], ))
                #printd("line: %s" % ([l[0][0], l[0][1], l[1][0], l[1][1]], ))
                #printd("IP: %s" % (ip, ))
                if mind is None:
                    mind = dist(rx, ry, ip[0], ip[1])
                    scanp = ip
                else:
                    newd = dist(rx, ry, ip[0], ip[1])
                    if mind > newd:
                        mind = newd
                        scanp = ip
            #else:
            #    printd("ray: %s" % ([rx, ry, rx + 1000*math.cos(self.fi), ry - 1000*math.sin(self.fi)], ))
            #    printd("line: %s" % ([l[0][0], l[0][1], l[1][0], l[1][1]], ))
            #    printd("IP: %s" % (ip, ))
            #    pause()

        if not scanp:
            #printd("None scanp")
            pass

        locks['sensorlock'].acquire()
        self.fi += self.w*deltatime
        self.dist = mind
        self.q = self.q[1:]
        self.q.append(scanp)
        locks['sensorlock'].release()
