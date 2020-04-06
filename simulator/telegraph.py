from threadjobs import createUDPSocket, launchAsThread
import sim
import struct
import time

TELEGRAPH_RECV_PORT = 54000

telegraphPoints = []
telegraphsocket = createUDPSocket(bind=True, port=TELEGRAPH_RECV_PORT)

def telegraphReset(locks):
    locks['telegraphlock'].acquire()
    telegraphPoints.clear()
    locks['telegraphlock'].release()

def stripheader(data):
    return data

def telegraphThreadJob(freq, locks):
    while not sim.exitsignal:
        data = stripheader(telegraphsocket.recv(4000))
        print("Telegraph got data %s" % (data, ))
        telegraphPointsBuffer = []
        while data:
            x = struct.unpack("d", data[0:8])
            y = struct.unpack("d", data[8:16])
            print("Telegraph got: %s" % ((x[0],y[0]), ))
            telegraphPointsBuffer.append((x[0],y[0]))
            data = data[16:]

        if telegraphPointsBuffer:
            locks['telegraphlock'].acquire()
            telegraphPoints.clear()
            for p in telegraphPointsBuffer:
                telegraphPoints.append(p)
            locks['telegraphlock'].release()
        time.sleep(1/freq)

def runTelegraphThread(locks, se):
    return launchAsThread(telegraphThreadJob, args=(1, locks))
