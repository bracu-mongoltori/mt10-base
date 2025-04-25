from processing_py import *
from math import radians, cos, sin, degrees, sqrt, pow

class Segment:
    def __init__(self, app, x=None, y=None, prev=None, length=None, offset=None, scl=10):
        self.app = app

        if prev is None:
            self.x = x
            self.y = y
            self.angle = radians(90)
            self.jnt = True
            self.scl = scl
        else:
            self.prev = prev
            self.angle = 0
            self.scl = scl
            self.len = length * scl
            self.target = 0
            self.offset = offset
            self.vOff = 0
            self.jnt = False
        self.limit = [0, 180]

    def show(self, main):
        app = self.app
        if self.jnt:
            app.noFill()
            app.stroke(0)
            app.circle(self.x, self.y, 15)
            return
        self.angle = radians(self.target)
        p1 = self.prev.getEnd()
        p2 = self.getEnd()
        app.noFill()
        app.stroke(0)
        if not main:
            app.stroke(128)
        app.line(p1[0], p1[1], p2[0], p2[1])
        app.circle(p2[0], p2[1], 15)

        app.stroke(0, 150)
        if main:
            app.arc(p1[0], p1[1], 100, 100, self.prev.getAng() + radians(180 - self.offset + self.limit[0]), self.prev.getAng() + radians(180 - self.offset + self.limit[1]))

            app.fill(255, 0, 0)
            app.textSize(15)
            app.text(f"{self.target:.0f}", p1[0] + 10, p1[1] - 10)

    def getEnd(self):
        if self.jnt:
            return [self.x, self.y]
        a = self.getAng()
        p = self.prev.getEnd()
        np = [p[0] - self.len * cos(a), p[1] - self.len * sin(a)]
        return np

    def getAng(self):
        if self.jnt:
            return 0
        ap = self.prev.getAng()
        return ap + self.angle - radians(self.offset)

    def write(self, t):
        self.target = t

scl = 10.0

len1, len2 = 15.5, 14
LIMIT1 = (30, 135)
LIMIT2 = (10, 160)
# LIMIT1 = (0, 180)
# LIMIT2 = (0, 270)

ik_pole_direction = 1

ang1, ang2 = 0, 0
_ang1, _ang2 = 0, 0
a1Offs = -60
a2Offs = 10
# a1Offs = 0
# a2Offs = 135

targetPx, targetPy = 0, 30

rngY = (0, 17)
rngX = (14, 24)


app = App(800, 500)
jnt = Segment(app, x=400, y=400)

sg1 = Segment(app, prev=jnt, length=len1, offset=a1Offs)
sg1.limit = LIMIT1
sg2 = Segment(app, prev=sg1, length=len2, offset=a2Offs)
sg2.limit = LIMIT2
sg2.vOff = 180

_sg1 = Segment(app, prev=jnt, length=len1, offset=a1Offs)
_sg1.limit = LIMIT1
_sg2 = Segment(app, prev=_sg1, length=len2, offset=a2Offs)
_sg2.limit = LIMIT2
_sg2.vOff = 180

def setAngles(a1, a2, _a1, _a2):
    global ang1, ang2, _ang1, _ang2
    ang1 = a1
    ang2 = a2
    _ang1 = _a1
    _ang2 = _a2

def setParams(len, limit, offs, dir=1, rng = None):
    global len1, len2, LIMIT1, LIMIT2, a1Offs, a2Offs, ik_pole_direction, rngX, rngY
    len1, len2 = len[0], len[1]
    LIMIT1 = limit[0]
    LIMIT2 = limit[1]
    a1Offs = offs[0]
    a2Offs = offs[1]
    ik_pole_direction = dir
    if rng:
        rngX = rng[0]
        rngY = rng[1]

def constrain(val, minv, maxv):
    return min(max(val, minv), maxv)

def dist(x1, y1, x2, y2):
    return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2))

def Loop():
    global ang1, ang2, _ang1, _ang2
    ang1 += 0.1
    ang2 += 0.1
    ang1 = constrain(ang1, LIMIT1[0], LIMIT1[1])
    ang2 = constrain(ang2, LIMIT2[0], LIMIT2[1])
    _ang1 = constrain(_ang1, LIMIT1[0], LIMIT1[1])
    _ang2 = constrain(_ang2, LIMIT2[0], LIMIT2[1])

    sg1.write(ang1)
    sg2.write(ang2)
    
    _sg1.write(_ang1)
    _sg2.write(_ang2)


def draw():
    app.background(255)
    Loop()

    p = headPos(ang1, ang2)
    pc = headPos(_ang1, _ang2)

    setTarget()

    for i in range(int(-app.width/scl), int(app.width/scl)):
        if i % 5 == 0:
            app.stroke(0, 50)
        else:
            app.stroke(200, 50)
        app.line(i * scl + jnt.x, 0, i * scl + jnt.x, app.height)
    for i in range(int(-app.height/scl), int(app.height/scl)):
        if i % 5 == 0:
            app.stroke(0, 50)
        else:
            app.stroke(200, 50)
        app.line(0, jnt.y - i * scl, app.width, jnt.y - i * scl)

    app.noFill()
    app.stroke(255, 100, 0)
    app.quad(rngX[0] * scl + jnt.x, jnt.y - rngY[0] * scl, rngX[1] * scl + jnt.x, jnt.y - rngY[0] * scl, rngX[1] * scl + jnt.x, jnt.y - rngY[1] * scl, rngX[0] * scl + jnt.x, jnt.y - rngY[1] * scl)

    app.stroke(0)
    app.line(jnt.x + targetPx * scl, jnt.y - targetPy * scl - 15, jnt.x + targetPx * scl, jnt.y - targetPy * scl + 15)
    app.line(jnt.x + targetPx * scl - 15, jnt.y - targetPy * scl, jnt.x + targetPx * scl + 15, jnt.y - targetPy * scl)

    d = dist(p[0], p[1], targetPx, targetPy)
    dc = dist(pc[0], pc[1], targetPx, targetPy)
    if d > 0.01:
        app.stroke(255, 0, 0)
        app.line(jnt.x + p[0] * scl, jnt.y - p[1] * scl, jnt.x + targetPx * scl, jnt.y - targetPy * scl)
        app.fill(255, 0, 0)
        app.text(f"{d:.2f}cm", jnt.x + ((p[0] + targetPx) / 2) * scl, jnt.y - ((p[1] + targetPy) / 2) * scl)
    if dc > 0.01:
        app.stroke(150, 0, 0, 150)
        app.line(jnt.x + pc[0] * scl, jnt.y - pc[1] * scl, jnt.x + targetPx * scl, jnt.y - targetPy * scl)
        app.fill(150, 0, 0, 150)
        app.text(f"{dc:.2f}cm", jnt.x + ((pc[0] + targetPx) / 2) * scl, jnt.y - ((pc[1] + targetPy) / 2) * scl)

    jnt.show(True)
    _sg1.show(False)
    _sg2.show(False)
    sg1.show(True)
    sg2.show(True)

    app.fill(255, 0, 0)
    app.textSize(20)
    # app.text(f"x: {targetPx:.2f} cm \ny: {targetPy:.2f} cm", 10, 20)



def headPos(ang1, ang2):
    a1 = ang1 - a1Offs
    a2 = a1 + ang2 - a2Offs

    rx = -(cos(radians(a1))*len1 + cos(radians(a2))*len2)
    ry = sin(radians(a1))*len1 + sin(radians(a2))*len2

    pos = (rx, ry)
    return pos

# def mouseDragged():
#     setTarget()


# def mousePressed():
#     setTarget()


# def mouseReleased():
#     setTarget()


def setTarget():
    global targetPx, targetPy
    targetPx = (app.mouseX - jnt.x)/scl
    targetPy = (jnt.y - app.mouseY)/scl

def rerun():
    draw()
    app.redraw()

if(__name__ == "__main__"):
    while(True):
        rerun()