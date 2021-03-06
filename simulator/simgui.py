import tkinter as tk
import threading
import itertools
import math

import sim
from geometry2 import mToCm, cmToM
from telegraph import telegraphPoints, telegraphReset
from threadjobs import locks

gui = []

def drawoval(canvas, radius, location, cfill="white", coutline="black"):
    canvas.create_oval(location[0] - radius, location[1] + radius, location[0] + radius, location[1] - radius, fill=cfill, outline=coutline)

def flatten(list_of_lists):
    """Flatten one level of nesting"""
    return itertools.chain.from_iterable(list_of_lists)

def drawgrid(canvas, offset, sizex=6,sizey=5):
    for x in range(0, sizex+1):
        canvas.create_line(offset + mToCm(x), canvas.height - offset, offset + mToCm(x), canvas.height - offset - mToCm(sizey), dash=(4,2), fill="ivory3")
        canvas.create_text(offset + mToCm(x), canvas.height - offset + 5, anchor=tk.N, text="%s"%(x,), fill="gray20")
    for y in range(0, sizey+1):
        canvas.create_line(offset, canvas.height - mToCm(y) - offset, offset + mToCm(sizex), canvas.height - offset - mToCm(y), dash=(4,2), fill="ivory3")
        canvas.create_text(offset- 5, canvas.height - mToCm(y) - offset, anchor=tk.E, text="%s"%(y,), fill="gray20")


def resetHandler(e, se):
    if e.char == 'r':
        se.robot.enqueueRequest('reset')
        telegraphReset(locks)

class SimGui(tk.Frame):
    def __init__(self, locks, se, master=None, canvasoffset= 100):
        super().__init__(master)
        self.master = master
        self.se = se
        self.locks = locks
        self.canvasoffset = canvasoffset
        self.create_widgets()

    def create_widgets(self):
        self.canvas = tk.Canvas(width=1000, height=700)
        self.canvas.width = 1000
        self.canvas.height = 700
        self.canvas.grid(row=0, column=0, rowspan=15)
        self.xLabel = tk.Label(self.master, text="Position x[m]:")
        self.yLabel = tk.Label(self.master, text="Position y[m]:")
        self.fiLabel = tk.Label(self.master, text="Angle [deg]:")
        self.fiRadLabel = tk.Label(self.master, text="Angle [rad]:")
        self.xLabelCurrent = tk.Label(self.master, text="N/A")
        self.yLabelCurrent = tk.Label(self.master, text="N/A")
        self.fiLabelCurrent = tk.Label(self.master, text="N/A")
        self.fiRadLabelCurrent = tk.Label(self.master, text="N/A")
        self.xLabel.grid(row=0,column=1)
        self.yLabel.grid(row=1,column=1)
        self.fiLabel.grid(row=2,column=1)
        self.fiRadLabel.grid(row=3,column=1)
        self.xLabelCurrent.grid(row=0,column=2)
        self.yLabelCurrent.grid(row=1,column=2)
        self.fiLabelCurrent.grid(row=2,column=2)
        self.fiRadLabelCurrent.grid(row=3,column=2)
        self.master.bind("<Key>", lambda e: resetHandler(e, self.se))


    @staticmethod
    def updateGUI():
        global gui
        gui.canvas.delete(tk.ALL)
        objects = [gui.se.areamap.boundary] + [obj.points for obj in gui.se.areamap.objects]
        objects = [[[mToCm(x) + gui.canvasoffset, gui.canvas.height - mToCm(y) - gui.canvasoffset] for x,y in o] for o in objects]
        for o in objects:
            flatcords = list(itertools.chain(*o))
            gui.canvas.create_polygon(flatcords, fill="white", outline="red")
        
        gui.locks['robotposlock'].acquire()
        robotx = gui.se.robot.pos.x
        roboty = gui.se.robot.pos.y
        robotrInCm = mToCm(gui.se.robot.r)
        robotfi = gui.se.robot.fi
        robotstartingfi  = gui.se.robot.resetCtor['fi'][1][0]
        robotstartingpos = gui.se.robot.resetCtor['pos'][1]
        robotw = gui.se.robot.w
        gui.locks['robotposlock'].release()

        gui.locks['sensorlock'].acquire()
        ipointq = gui.se.robot.lidar.q # intersection point queue, should be copied, but I don't care enough
        lidarfi = gui.se.robot.lidar.fi
        gui.locks['sensorlock'].release()

        # Update labels
        gui.xLabelCurrent.configure(text="{}".format(robotx))
        gui.yLabelCurrent.configure(text="{}".format(roboty))
        gui.fiLabelCurrent.configure(text="{}".format(robotfi*180/math.pi))
        gui.fiRadLabelCurrent.configure(text="{}".format(robotfi))

        # Grid 
        drawgrid(gui.canvas, gui.canvasoffset)

        # Robot body
        robotxInCm = mToCm(robotx)
        robotyInCm = mToCm(roboty)
        drawoval(gui.canvas, robotrInCm, [robotxInCm + gui.canvasoffset, gui.canvas.height - robotyInCm - gui.canvasoffset])

        # Lidar points
        #print("drawing ipointq %s" % (ipointq))
        for p in ipointq:
            drawoval(gui.canvas, 2, [mToCm(p[0]) + gui.canvasoffset, gui.canvas.height - mToCm(p[1]) - gui.canvasoffset])

        gui.locks['telegraphlock'].acquire()
        for tp in telegraphPoints:
            tpxRot = math.cos(robotstartingfi)*tp[0] - math.sin(robotstartingfi)*tp[1]
            tpyRot = math.sin(robotstartingfi)*tp[0] + math.cos(robotstartingfi)*tp[1]
            tpxAbs = robotstartingpos[0] + tpxRot
            tpyAbs = robotstartingpos[1] + tpyRot
            drawoval(gui.canvas, 10, [mToCm(tpxAbs) + gui.canvasoffset, gui.canvas.height - mToCm(tpyAbs) - gui.canvasoffset], cfill="red")
        gui.locks['telegraphlock'].release()
            

        # lidar line
        # gui.canvas.create_line(robotxInCm + gui.canvasoffset, robotyInCm + gui.canvasoffset, gui.canvasoffset + robotxInCm + 1000*math.cos(lidarfi), gui.canvasoffset + robotyInCm - 1000*math.sin(lidarfi) )

        # Orientation vector
        gui.canvas.create_line(robotxInCm + gui.canvasoffset, gui.canvas.height - robotyInCm - gui.canvasoffset, gui.canvasoffset +  robotxInCm + robotrInCm*math.cos(robotfi), gui.canvas.height - gui.canvasoffset - robotyInCm - robotrInCm*math.sin(robotfi))

        gui.master.after(16, SimGui.updateGUI)

def createExitFcn(root):
    def exitfcn():
        if sim.exitsignal:
            root.quit()
        else:
            root.after(16, exitfcn)
    return exitfcn

def runGUI(locks, se):
    global gui
    root = tk.Tk()
    gui = SimGui(locks, se, master=root)
    root.after(16, SimGui.updateGUI)
    root.after(16, createExitFcn(root))
    root.mainloop()
