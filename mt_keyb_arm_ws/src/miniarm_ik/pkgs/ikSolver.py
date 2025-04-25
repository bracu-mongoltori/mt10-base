from math import atan2, degrees, radians, acos, cos, sin, sqrt

def dist(p1, p2):
    return sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

def headPos(ang, len, offs):
    a1 = ang[0] - offs[0]
    a2 = a1 + ang[1] - offs[1]

    rx = (cos(radians(a1))*len[0] + cos(radians(a2))*len[1])
    ry = sin(radians(a1))*len[0] + sin(radians(a2))*len[1]

    pos = (rx, ry)
    return pos

def calcAngles(targetP, len, limits, offs, ik_dir = 1):
    x = targetP[0]
    y = targetP[1]

    head_position = (0, 0)

    sqd = x**2 + y**2
    d = sqrt(sqd)
    ang = degrees(atan2(y, x))
    if(ang < 0):
        ang = ang + 360

    if (d > len[0] + len[1]):
        a1 = ang + offs[0]
        a2 = offs[1]
        _a1 = max(min(a1, limits[0][1]), limits[0][0])
        _a2 = max(min(a2, limits[1][1]), limits[1][0])
        head_position = headPos((_a1, _a2), len, offs)
        return {
            "ang1": _a1*1.0,
            "ang2": _a2*1.0,
            "head_position": head_position,
            "dist": dist(targetP, head_position),
            "ang1_full": a1*1.0,
            "ang2_full": a2*1.0
        }

    aB = degrees(acos( (len[0]**2 + sqd - len[1]**2) / (2 * len[0] * d)))

    aT = degrees(acos( (len[0]**2 + len[1]**2 - sqd) / (2 * len[0] * len[1]) ))
    a1 = ang - aB*ik_dir + offs[0]
    a2 = offs[1] + (180 - aT)*ik_dir
    _a1 = max(min(a1, limits[0][1]), limits[0][0])
    _a2 = max(min(a2, limits[1][1]), limits[1][0])
    head_position = headPos((_a1, _a2), len, offs)
    return {
        "ang1": _a1*1.0,
        "ang2": _a2*1.0,
        "head_position": head_position,
        "dist": dist(targetP, head_position),
        "ang1_full": a1*1.0,
        "ang2_full": a2*1.0
    }
