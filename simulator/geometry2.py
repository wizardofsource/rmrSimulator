import math

# Return intersection point of two lines segments, return None if it doesnt not exit.
def llintersectpoint(x1,y1,x2,y2,x3,y3,x4,y4):
    den = (x1-x2)*(y3-y4) - (y1 - y2)*(x3 - x4)
    if not den: # Parallel segments
        #print("Parallel") 
        return None
    t = 1/den * ( (x1-x3)*(y3-y4) - (y1-y3)*(x3-x4) )
    u = -1/den * ( (x1-x2)*(y1-y3) - (y1-y2)*(x1-x3) )

    if 0 <= t  and t <= 1 and 0 <= u and u <= 1: # Segments do intersect
        return [x1 + t*(x2-x1), y1 + t*(y2-y1)] # Intersection point
    #print("Non intersecting") 
    return None # No intersection on segments

def dist(x1,y1,x2,y2):
    return math.sqrt((x1-x2)**2 + (y1-y2)**2)

def mmToM(mm):
    return mm/1000

def mToCm(m):
    return m*100

def cmToMm(cm):
    return cm*10

def cmToM(cm):
    return cm/100

def mToMm(m):
    return m*1000

def pointToLineDistance(point, line):
    x3,y3 = point
    x1,y1 = line[0]
    x2,y2 = line[1]
    px = x2-x1
    py = y2-y1
    norm = px*px + py*py
    u =  ((x3 - x1) * px + (y3 - y1) * py) / float(norm)
    if u > 1:
        u = 1
    elif u < 0:
        u = 0
    x = x1 + u * px
    y = y1 + u * py
    dx = x - x3
    dy = y - y3
    dist = (dx*dx + dy*dy)**.5
    return dist
