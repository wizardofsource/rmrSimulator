import simgui
from robot import Robot, robotDefaultCtor
import simenv
import threadjobs
from threadjobs import runLidarSenderThread, runSensorSenderThread, runRobotCommandReaderThread, runUpdateThread, runCLIReaderThread
import debug
from debug import printd, pause

exitsignal = False

if __name__ == '__main__':
    debug.debug = False
    with open("priestor.txt", 'r') as f:
        simenv.se = simenv.SimEnvironment(simenv.MapParser.parse(f.read()), Robot(robotDefaultCtor))
    runningThreads = []
    threadJobs = [runLidarSenderThread, runSensorSenderThread, runRobotCommandReaderThread, runUpdateThread, runCLIReaderThread]
    threadArgs = [[threadjobs.locks], [threadjobs.locks, simenv.se], [simenv.se], [threadjobs.locks, simenv.se], [threadjobs.locks, simenv.se]]
    for tj, a in zip(threadJobs, threadArgs):
        runningThreads.append(tj(*a))

    simgui.runGUI(threadjobs.locks, simenv.se)

    for t in runningThreads:
        t.join()

    sys.exit(0)
