import socket
import time
import threading
import struct
import math 
import sys

import simenv
import sim

from debug import printd, pause
from geometry2 import mToMm

HOST = ''
LIDARPORT_SEND = 52999
LIDARPORT_RECV = 5299
ROBOTPORT_SEND = 53000
ROBOTPORT_RECV = 5300

laserdatastructstr = "idd"
extrareqstructstr = "BBBBBBIIIBIII"
robotdatastructstr = "QhHHQbhHQbQQQQH" + extrareqstructstr

lidartimer = 0
locks = { 'robotposlock' : threading.Lock(), 'sensorlock' : threading.Lock(), 'telegraphlock' : threading.Lock()}

def lidarjob(freq, soc, locks, chunk):
    global lidartimer
    num = 0
    packeddata = b""
    while not sim.exitsignal: 
        lidartimer = time.time()
        locks['sensorlock'].acquire()
        fi = simenv.se.robot.lidar.fi 
        dist = simenv.se.robot.lidar.dist
        # printd("LIDAR: FI: %s DIST: %s" %(fi, dist))
        locks['sensorlock'].release()
        locks['robotposlock'].acquire()
        robotfi = simenv.se.robot.fi 
        locks['robotposlock'].release()
        packeddata += struct.pack(laserdatastructstr, 1, fi*180/math.pi, mToMm(dist)) # here dist is in cm but mm are required also looks like rotation is swapped 
        if num == chunk:
            num = 0
            soc.sendto(packeddata, ('localhost', LIDARPORT_SEND))
            packeddata = b""
        else:
            num+=1
        time.sleep(1/freq)
         
def createUDPSocket(bind=False, port=0):
    if bind:
        soc = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        soc.bind(("127.0.0.1", port))
        return soc
    return socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def launchAsThread(f, args):
    th = threading.Thread(target=f, args=args)
    th.start()
    return th

def runLidarSenderThread(locks):
    lidarsocket = createUDPSocket()
    return launchAsThread(lidarjob, args=(360*7.9,lidarsocket, locks, 360))

def robotCommandReaderJob(freq, sock, se):
    sock.bind(('127.0.0.1', ROBOTPORT_RECV))
    printd("watining for cmds.")
    while not sim.exitsignal:
        cmd = sock.recv(16)
        printd("got: %s ", cmd)
        simenv.se.robot.execute(list(cmd), cmd) 
        time.sleep(1/freq)
     
def runRobotCommandReaderThread(se):
    commandsocket = createUDPSocket()
    return launchAsThread(robotCommandReaderJob, (50, commandsocket, se))


def updateJob(freq, se, locks):
    oldtime = time.time()
    while not sim.exitsignal:
        newtime = time.time()
        simenv.se.robot.update(newtime - oldtime, locks, se)
        oldtime = newtime
        time.sleep(1/freq)

def runUpdateThread(locks, se):
    return launchAsThread(updateJob, args=(15000, se, locks))

def getBasicSensorDataFeedback(sock, lock, se):
    #printd("IRC Increments: %s %s" % (simenv.se.robot.leftIRC.getIncrements(), simenv.se.robot.rightIRC.getIncrements()))
    return struct.pack("<HbbbHHbbbbbb",1,2,3,4, simenv.se.robot.leftIRC.getIncrements(), simenv.se.robot.rightIRC.getIncrements(), 7, 8, 9, 10, 11, 12)

def getInertialSensorFeedback(sock, lock, se):
    #printd("GyroAngle: %s GyroAngleRate: %s" % (simenv.se.robot.gyro.getAngle(), simenv.se.robot.gyro.getAngleRate()))
    return struct.pack("<hHbbb", 0, 0, 0, 0, 0)

def addHeader(packeddata):
    return struct.pack("<BBB", 0xaa, 0x56, len(packeddata)) + packeddata  # really should be 0x55 instead of 0x56

def wrapSubPayload(header, payload):
    # printd("len payload %s" %(len(payload), ))
    # printd("payload %s" %(list(payload), ))
    return struct.pack("<BB", header, len(payload)) + payload

def addChecksum(datawithheader):
    cs = 0
    for b in datawithheader[2:]:
        cs = cs ^ b
    return datawithheader + struct.pack("B", cs)

def addChecksum2(datawithheader):
    cs = 0
    #printd("datawithheader: %s" %(list(datawithheader),))
    for b in datawithheader:
        cs = cs ^ b
    #printd("cs: %s" % (cs, ))
    return datawithheader + struct.pack("B", cs % 256)

def sensorSendingJob(freq, locks, se, sock):
    while not sim.exitsignal:
        payload = wrapSubPayload(1, getBasicSensorDataFeedback(sock, locks, se)) + wrapSubPayload(4, getInertialSensorFeedback(sock, locks, se)) # for some reason they don't use the standard header (probably because UDP)
        # for some reason they also add payload without checksum to payload with checksum w/e
        payload = addChecksum2(struct.pack("B", len(payload))  + payload)

        #printd("payload %s", list(payload))
        sock.sendto(payload, ('localhost', ROBOTPORT_SEND))
        time.sleep(1/freq)


def runSensorSenderThread(locks, se):
    sock = createUDPSocket()
    return launchAsThread(sensorSendingJob, args=(50, locks, se, sock))

def CLIReaderJob(freq, locks, se):
    printd("CLI reader job got args: %s %s %s" % (freq, locks, se))
    while not sim.exitsignal:
        cmd = input(" Command :> ")
        if cmd == "reset":
            simenv.se.robot.enqueueRequest('reset')
        if cmd == "exit":
            sim.exitsignal = True
        time.sleep(1/freq)

def runCLIReaderThread(locks, se):
    return launchAsThread(CLIReaderJob, (10, locks, se))
