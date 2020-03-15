from geometry2 import llintersectpoint, dist, mmToM, mToCm, cmToMm, pointToLineDistance, cmToM, mToMm

se = []

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
        #printd("calclines of points: %s" % (self.points, ))
        for p1, p2 in zip(self.points, self.points[1:] + [self.points[0]]):
            #printd("Creating line %s" % ([p1,p2],))
            self.lines.append([p1,p2])

    def __str__(self):
        return "CO points: " + str(self.points) + "\nCO lines: " + str(self.lines)

class Line:
    def __init__(self, start, end):
        if (len(start) != 2 or len(end) != 2):
            printd("Line wrong arguments")
            os.exit(1)
        self.start = start
        self.end = end

class AreaMap:
    def __init__(self, boundary):
        self.boundary = boundary
        self.boundarylines = []
        for p1, p2 in zip(self.boundary, self.boundary[1:] + [self.boundary[0]]):
            self.boundarylines.append([p1,p2])
        #printd("created boundary lines: %s" %(self.boundarylines, ))
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
                #printd("parsed %s", (points, ))
            if i==0:
                amap = AreaMap(points)
            else:
                if points:
                    amap.addObject(CollisionObject(points)) 
        return amap
