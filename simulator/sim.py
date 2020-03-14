import socket
import sys
import threading
import struct
import time
import math
import time
import tkinter
import os

from copy import deepcopy
from simgui import runGUI
from geometry2 import llintersectpoint, dist, mmToM, mToCm, cmToMm, pointToLineDistance, cmToM, mToMm

def pause():
    p = input("Press the <Enter> key to continue...")

HOST = ''
LIDARPORT_SEND = 52999
LIDARPORT_RECV = 5299
ROBOTPORT_SEND = 53000
ROBOTPORT_RECV = 5300

laserdatastructstr = "idd"
extrareqstructstr = "BBBBBBIIIBIII"
robotdatastructstr = "QhHHQbhHQbQQQQH" + extrareqstructstr

lidartimer = 0

locks = { 'robotposlock' : threading.Lock(), 'sensorlock' : threading.Lock()}

class SimEnvironment:
    def __init__(self, areamap, robot):
        self.areamap = areamap
        self.robot = robot

class CollisionObject:
    def __init__(self, points):
        self.points = points
        self.lines = []
        self.calclines()

    def calclines(self):
        #print("calclines of points: %s" % (self.points, ))
        for p1, p2 in zip(self.points, self.points[1:] + [self.points[0]]):
            #print("Creating line %s" % ([p1,p2],))
            self.lines.append([p1,p2])

    def __str__(self):
        return "CO points: " + str(self.points) + "\nCO lines: " + str(self.lines)

class Line:
    def __init__(self, start, end):
        if (len(start) != 2 or len(end) != 2):
            print("Line wrong arguments")
            os.exit(1)
        self.start = start
        self.end = end

class AreaMap:
    def __init__(self, boundary):
        self.boundary = boundary
        self.boundarylines = []
        for p1, p2 in zip(self.boundary, self.boundary[1:] + [self.boundary[0]]):
            self.boundarylines.append([p1,p2])
        #print("created boundary lines: %s" %(self.boundarylines, ))
        #pause()
        self.objects = []

    def addObject(self, colObj):
        self.objects.append(colObj)

    def __str__(self):
        strrep = str(self.boundary)
        for obj in self.objects:
            strrep += "\n" + str(obj)
        return strrep


class MapParser:
    @staticmethod
    def parse(instr):
        strlines = instr.splitlines()

        for i, strline in enumerate(strlines):
            datastr, *commentstr = strline.split("/")
            numpointsAndPoints = datastr.split(" ")
            numpoints = numpointsAndPoints[0]
            pointsStr = numpointsAndPoints[1:]

            points = []
            for ps in pointsStr:
                if not ps:
                    continue
                xystr = ps[1:-1].split(",")
                x = cmToM(float(xystr[0])) # convert from centimeters to meters 
                y = cmToM(float(xystr[1])) 
                points.append([x,y])
                #print("parsed %s", (points, ))
            if i==0:
                amap = AreaMap(points)
            else:
                if points:
                    amap.addObject(CollisionObject(points)) 
        return amap

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
                #print("found ip: %s" % ([rx, ry, rx + 1000*math.cos(self.fi), ry - 1000*math.sin(self.fi)], ))
                #print("line: %s" % ([l[0][0], l[0][1], l[1][0], l[1][1]], ))
                #print("IP: %s" % (ip, ))
                if mind is None:
                    mind = dist(rx, ry, ip[0], ip[1])
                    scanp = ip
                else:
                    newd = dist(rx, ry, ip[0], ip[1])
                    if mind > newd:
                        mind = newd
                        scanp = ip
            #else:
            #    print("ray: %s" % ([rx, ry, rx + 1000*math.cos(self.fi), ry - 1000*math.sin(self.fi)], ))
            #    print("line: %s" % ([l[0][0], l[0][1], l[1][0], l[1][1]], ))
            #    print("IP: %s" % (ip, ))
            #    pause()

        if not scanp:
            #print("None scanp")
            pass

        locks['sensorlock'].acquire()
        self.fi += self.w*deltatime
        self.dist = mind
        self.q = self.q[1:]
        self.q.append(scanp)
        locks['sensorlock'].release()

class LeftRightPair:
    def __init__(self, left, right):
        self.left = left;
        self.right = right;

class XYPair:
    def __init__(self, x, y):
        self.x = x
        self.y = y


class Robot:
    def __init__(self, r=0.035, d=0.23, pos=XYPair(1,1), speeds=LeftRightPair(0,0), fi=0, w=0, gyro = Gyro(), leftIRC = IRCSensor(), rightIRC = IRCSensor(), lidar = Lidar(0, 0, 2*math.pi*7.9)): # w should be 2*pi*7.9
        self.r = r
        self.d = d
        self.fi = fi
        self.w = w
        self.pos = pos
        self.speeds = speeds

        self.gyro = gyro
        self.leftIRC = leftIRC
        self.rightIRC = rightIRC
        self.lidar = lidar

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
                    print("Collision d=%s" %(d, ))
                    return True
        return False


    def update(self, deltatime, locks, se):
        locks['robotposlock'].acquire()
        jointspeed = (self.speeds.right + self.speeds.left) / 2
        #print("Jointed speed: %s" % (jointspeed,))
        rollbackx = self.pos.x;
        rollbacky = self.pos.y;
        self.pos.x += math.cos(self.fi)*jointspeed*deltatime
        self.pos.y += math.sin(self.fi)*jointspeed*deltatime
        # print("FI : %s " %(self.fi))
        # Comment this if it's too slow
        if self.isCollision(se):
            print("COLLISION")
            self.pos.x = rollbackx;
            self.pos.y = rollbacky;
        #print("ROBOT POS: %s %s" % (self.pos.x , self.pos.y))
        self.w = (self.speeds.right - self.speeds.left)/self.d
        self.fi += self.w*deltatime
        locks['robotposlock'].release()
        # print("ROBOT W: %s FI: %s" % (self.w, self.fi))

        self.lidar.update(deltatime, self, se.areamap, locks)
        self.gyro.update(self.w, self.fi)

        locks['sensorlock'].acquire()
        self.leftIRC.update(deltatime*(self.speeds.left))
        self.rightIRC.update(deltatime*(self.speeds.right))
        locks['sensorlock'].release()


    def execute(self, cmd, cmdstr):
        if cmd[:9] == [0xaa, 0x55, 0x0A, 0x0c, 0x02, 0xf0, 0x00, 0x01, 0x04]:
            hdr1, hdr2, speed, radius = struct.unpack("<Qbhh", cmdstr[:13])

            speedinm = mmToM(speed)
            print("SPEEDINM : %s" % (speedinm, ))
            if radius == 1:
                print("only rotate %s" % (speedinm, ))
                self.onlyRotate(speedinm)
            elif radius == 0:
                print("only translate")
                self.onlyTranslate(speedinm)
            elif radius > 1 or radius < -1: 
                radiusinm = mmToM(radius)
                print("arc %s %s" % (speedinm, radius))
                self.arc(speedinm, radiusinm)
            else:
                print("Malformed command, stopping...")
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

def lidarjob(freq, soc, locks, chunk):
    global lidartimer
    num = 0
    packeddata = b""
    while True: 
        lidartimer = time.time()
        locks['sensorlock'].acquire()
        fi = se.robot.lidar.fi 
        dist = se.robot.lidar.dist
        # print("LIDAR: FI: %s DIST: %s" %(fi, dist))
        locks['sensorlock'].release()
        locks['robotposlock'].acquire()
        robotfi = se.robot.fi 
        locks['robotposlock'].release()
        packeddata += struct.pack(laserdatastructstr, 1, fi*180/math.pi, mToMm(dist)) # here dist is in cm but mm are required also looks like rotation is swapped 
        if num == chunk:
            num = 0
            soc.sendto(packeddata, ('localhost', LIDARPORT_SEND))
            packeddata = b""
        else:
            num+=1
        time.sleep(1/freq)
         
def createUDPSocket():
    return socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def runLidarSender(locks):
    lidarsocket = createUDPSocket()
    ls = threading.Thread(target=lidarjob, args=( 360*7.9,lidarsocket, locks, 360)) # freq should = 360*7.9, it is low only for testing
    ls.start()
    return ls

#def runRobotSender():
#    #threading.Thread(target=robotjob, args=(robotfreq,))
#    pass

def commandReaderJob(freq, sock, se):
    sock.bind(('127.0.0.1', ROBOTPORT_RECV))
    print("watining for cmds.")
    while True:
        cmd = sock.recv(16)
        print("got: %s ", cmd)
        se.robot.execute(list(cmd), cmd) 
        time.sleep(1/freq)
     
def runCommandReader(se):
    commandsocket = createUDPSocket()
    th = threading.Thread(target=commandReaderJob, args=(50, commandsocket, se))
    th.start()
    return th


def updateJob(freq, se, locks):
    oldtime = time.time()
    while True:
        newtime = time.time()
        se.robot.update(newtime - oldtime, locks, se)
        oldtime = newtime
        time.sleep(1/freq)

def runUpdateThread(locks, se):
    th = threading.Thread(target=updateJob, args=(15000, se, locks))
    th.start()
    return th

def getBasicSensorDataFeedback(sock, lock, se):
    #print("IRC Increments: %s %s" % (se.robot.leftIRC.getIncrements(), se.robot.rightIRC.getIncrements()))
    return struct.pack("<HbbbHHbbbbbb",1,2,3,4, se.robot.leftIRC.getIncrements(), se.robot.rightIRC.getIncrements(), 7, 8, 9, 10, 11, 12)

def getInertialSensorFeedback(sock, lock, se):
    #print("GyroAngle: %s GyroAngleRate: %s" % (se.robot.gyro.getAngle(), se.robot.gyro.getAngleRate()))
    return struct.pack("<hHbbb", 0, 0, 0, 0, 0)

def addHeader(packeddata):
    return struct.pack("<BBB", 0xaa, 0x56, len(packeddata)) + packeddata  # really should be 0x55 instead of 0x56

def wrapSubPayload(header, payload):
    # print("len payload %s" %(len(payload), ))
    # print("payload %s" %(list(payload), ))
    return struct.pack("<BB", header, len(payload)) + payload

def addChecksum(datawithheader):
    cs = 0
    for b in datawithheader[2:]:
        cs = cs ^ b
    return datawithheader + struct.pack("B", cs)

def addChecksum2(datawithheader):
    cs = 0
    #print("datawithheader: %s" %(list(datawithheader),))
    for b in datawithheader:
        cs = cs ^ b
    #print("cs: %s" % (cs, ))
    return datawithheader + struct.pack("B", cs % 256)

def sensorSendingJob(freq, locks, se, sock):
    while True:
        payload = wrapSubPayload(1, getBasicSensorDataFeedback(sock, locks, se)) + wrapSubPayload(4, getInertialSensorFeedback(sock, locks, se)) # for some reason they don't use the standard header (probably because UDP)
        # for some reason they also add payload without checksum to payload with checksum w/e
        payload = addChecksum2(struct.pack("B", len(payload))  + payload)

        #print("payload %s", list(payload))
        sock.sendto(payload, ('localhost', ROBOTPORT_SEND))
        time.sleep(1/freq)


def runSensorSender(locks, se):
    sock = createUDPSocket()
    th = threading.Thread(target=sensorSendingJob, args=(50, locks, se, sock))
    th.start()
    return th

if __name__ == '__main__':
    se = []
    with open("newmap.txt", 'r') as f:
        se = SimEnvironment(MapParser.parse(f.read()), Robot())
    print(se.areamap)
    print(se.areamap.boundarylines)
    runLidarSender(locks)
    runSensorSender(locks, se)
    runCommandReader(se)
    runUpdateThread(locks, se)
    runGUI(locks, se) # this blocks
