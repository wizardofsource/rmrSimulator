from simgui import runGUI
from robot import Robot, robotDefaultCtor
import simenv
import threadjobs
from threadjobs import runLidarSenderThread, runSensorSenderThread, runRobotCommandReaderThread, runUpdateThread, runCLIReaderThread
import debug
from debug import printd, pause
debug.debug = False

if __name__ == '__main__':
    with open("priestor.txt", 'r') as f:
        simenv.se = simenv.SimEnvironment(simenv.MapParser.parse(f.read()), Robot(robotDefaultCtor))
    # printd(se.areamap)
    # printd(se.areamap.boundarylines)
    runLidarSenderThread(threadjobs.locks)
    runSensorSenderThread(threadjobs.locks, simenv.se)
    runRobotCommandReaderThread(simenv.se)
    runUpdateThread(threadjobs.locks, simenv.se)
    runCLIReaderThread(threadjobs.locks, simenv.se)
    runGUI(threadjobs.locks, simenv.se) # this blocks, must be last
