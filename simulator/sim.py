#!/usr/bin/python3
import simgui
from robot import Robot, robotDefaultCtor, fillFromIni
import simenv
import threadjobs
from threadjobs import runLidarSenderThread, runSensorSenderThread, runRobotCommandReaderThread, runUpdateThread, runCLIReaderThread
from telegraph import runTelegraphThread
import debug
from debug import printd, pause
from inifiles import readini
import sys

exitsignal = False

if __name__ == '__main__':
    debug.debug = False
    inifile = readini("sim.config")
    print("inifiledict is : %s" %(inifile,))
    with open(inifile['environment']['mapfile'], 'r') as f:
        simenv.se = simenv.SimEnvironment(simenv.MapParser.parse(f.read()), Robot(fillFromIni(robotDefaultCtor, inifile)))
    runningThreads = []
    threadJobs = [runLidarSenderThread, runSensorSenderThread, runRobotCommandReaderThread, runUpdateThread, runTelegraphThread] #, runCLIReaderThread]
    threadArgs = [[threadjobs.locks], [threadjobs.locks, simenv.se], [simenv.se], [threadjobs.locks, simenv.se], [threadjobs.locks, simenv.se], [threadjobs.locks, simenv.se]]
    for tj, a in zip(threadJobs, threadArgs):
        runningThreads.append(tj(*a))

    simgui.runGUI(threadjobs.locks, simenv.se)

    for t in runningThreads:
        t.join()

    sys.exit(0)
