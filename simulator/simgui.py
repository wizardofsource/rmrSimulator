import tkinter as tk
import threading
import itertools
import math

from geometry2 import mToCm

gui = []

def drawoval(canvas, radius, location):
    canvas.create_oval(location[0] - 0.7*radius, location[1] + 0.7*radius, location[0] + 0.7*radius, location[1] - 0.7*radius)

def flatten(list_of_lists):
    """Flatten one level of nesting"""
    return itertools.chain.from_iterable(list_of_lists)

class SimGui(tk.Frame):
    def __init__(self, locks, se, master=None, canvasoffset= 100):
        super().__init__(master)
        self.master = master
        self.se = se
        self.locks = locks
        self.canvasoffset = canvasoffset
        self.create_widgets()

    def create_widgets(self):
        self.canvas = tk.Canvas(width=1000, height=1000)
        self.canvas.grid(row=0, column=0, rowspan=15)
        self.xLabel = tk.Label(self.master, text="Position x[cm]:")
        self.yLabel = tk.Label(self.master, text="Position y[cm]:")
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

    @staticmethod
    def updateGUI():
        global gui
        gui.canvas.delete(tk.ALL)
        objects = [gui.se.areamap.boundary] + [obj.points for obj in gui.se.areamap.objects]
        objects = [[[x + gui.canvasoffset, y + gui.canvasoffset] for x,y in o] for o in objects]
        for o in objects:
            flatcords = list(itertools.chain(*o))
            gui.canvas.create_polygon(flatcords, fill="white", outline="red")
        
        gui.locks['robotposlock'].acquire()
        robotxInCm = mToCm(gui.se.robot.pos.x)
        robotyInCm = mToCm(gui.se.robot.pos.y)
        robotdInCm = mToCm(gui.se.robot.d)
        robotfi = gui.se.robot.fi
        robotw = gui.se.robot.w
        gui.locks['robotposlock'].release()

        gui.locks['sensorlock'].acquire()
        ipointq = gui.se.robot.lidar.q # intersection point queue
        lidarfi = gui.se.robot.lidar.fi
        gui.locks['sensorlock'].release()

        # Update labels
        gui.xLabelCurrent.configure(text="{}".format(robotxInCm))
        gui.yLabelCurrent.configure(text="{}".format(robotyInCm))
        gui.fiLabelCurrent.configure(text="{}".format(robotfi*180/math.pi))
        gui.fiRadLabelCurrent.configure(text="{}".format(robotfi))

        # Robot body
        drawoval(gui.canvas, robotdInCm/2, [robotxInCm + gui.canvasoffset, robotyInCm + gui.canvasoffset])

        # Lidar points
        #print("drawing ipointq %s" % (ipointq))
        for p in ipointq:
            drawoval(gui.canvas, 2, [v + gui.canvasoffset for v in p])

        # lidar line
        # gui.canvas.create_line(robotxInCm + gui.canvasoffset, robotyInCm + gui.canvasoffset, gui.canvasoffset + robotxInCm + 1000*math.cos(lidarfi), gui.canvasoffset + robotyInCm - 1000*math.sin(lidarfi) )

        # Orientation vector
        gui.canvas.create_line(robotxInCm + gui.canvasoffset, robotyInCm + gui.canvasoffset, gui.canvasoffset +  robotxInCm + 23*math.cos(robotfi), gui.canvasoffset + robotyInCm - 23*math.sin(robotfi))

        gui.master.after(16, SimGui.updateGUI)

def runGUI(locks, se):
    global gui
    root = tk.Tk()
    gui = SimGui(locks, se, master=root)
    root.after(16, SimGui.updateGUI)
    root.mainloop()
