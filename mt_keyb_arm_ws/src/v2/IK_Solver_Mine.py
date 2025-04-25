import time
import serial
from math import atan2, degrees, radians, pi, acos, cos, sin

len1:float = 15.5
len2:float = 14.0

LIMIT1:int = [30, 135]
LIMIT2:int = [10, 160]
a1Offs:int = -60
a2Offs:int = 10

# double ang1, ang2;
# double cAng1, cAng2;

targetP = {"x": 21.4, "y": 13.8}
ser = None
for i in range(10):
    try:
        ser = serial.Serial(f'/dev/ttyACM{i}', 9600)
        print(f"Connected to /dev/ttyACM{i}")
        break
    except:
        print(f"Failed to connect to /dev/ttyACM{i}")
        time.sleep(0.1)

if(ser == None):
    print("Failed to connect to arduino")

def dist(p1, p2):
    return ((p1["x"] - p2["x"])**2 + (p1["y"] - p2["y"])**2)**0.5

def headPos(ang, len, offs):
    a1 = ang[0] - offs[0]
    a2 = a1 + ang[1] - offs[1]

    rx = -(cos(radians(a1))*len[0] + cos(radians(a2))*len[1])
    ry = sin(radians(a1))*len[0] + sin(radians(a2))*len[1]

    pos = {"x": rx, "y": ry}
    print(pos)
    return pos

def calcAngles(targetP, len, limits, offs, ik_dir = 1):
    x = targetP["x"]
    y = targetP["y"]

    head_position = {"x": 0, "y": 0}

    sqd = targetP["x"]**2 + targetP["y"]**2
    d = sqd**0.5
    ang = degrees(atan2(y, -x))
    if(ang < 0):
        ang = ang + 360

    if (d > len[0] + len[1]):
        a1 = ang + offs[0]
        a2 = offs[1]
        a1 = max(min(a1, limits[0][1]), limits[0][0])
        a2 = max(min(a2, limits[1][1]), limits[1][0])
        head_position = headPos((a1, a2), len, offs)
        return a1, a2, head_position, dist(targetP, head_position)

    aB = degrees(acos( (len[0]**2 + sqd - len[1]**2) / (2 * len[0] * d)))

    aT = degrees(acos( (len[0]**2 + len[1]**2 - sqd) / (2 * len[0] * len[1]) ))
    a1 = ang - aB*ik_dir + offs[0]
    a2 = offs[1] + (180 - aT)*ik_dir
    a1 = max(min(a1, limits[0][1]), limits[0][0])
    a2 = max(min(a2, limits[1][1]), limits[1][0])
    head_position = headPos((a1, a2), len, offs)
    return a1, a2, head_position, dist(targetP, head_position)

while True:
    targetP["x"] = float(input("Enter the target x position (cm): "))
    targetP["y"] = float(input("Enter the target y position (cm): "))

    ang1, ang2, head, dst = calcAngles(targetP, (len1, len2), (LIMIT1, LIMIT2), (a1Offs, a2Offs))
    print(f"Angles: {int(ang1)}, {int(ang2)}")
    print(f"Head Pos: {head}")
    print(f"Distance: {dst}")
    if(ser != None):
        ser.write(f"{int(ang1)} {int(ang2)}\n".encode())