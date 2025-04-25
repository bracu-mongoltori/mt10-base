import numpy as np
from pkgs.params import key_pos
import pkgs.params as params

# keyb_corners = [[0, 0], [46.0, 0], [46.0, 14.45], [0, 14.45]] #constant
# detected_corners =  [[98, 280], [611, 280], [629, 433], [74, 434]] #detection
# screen_corners = [[129, 29], [696, 35], [771, 502], [36, 493]] #tuning
# world_corners = [[-30.0, 50.0], [30.0, 50.0], [30.0, 10.0], [-30.0, 10.0]] #tuning

keyb_corners = [[0, 0], [46.0, 0], [46.0, 14.45], [0, 14.45]] #constant
detected_corners =  [[84, 127], [704, 131], [706, 320], [77, 317]] #detection
screen_corners = [[46, 136], [768, 145], [773, 410], [26, 398]] #tuning
world_corners = [[-30.0, 27.0], [25.0, 27.0], [25.0, 7.0], [-30.0, 7.0]] #tuning

H_k2d = None #homography: keyboard (real world) to detected (on-screen)
H_s2w = None #homography: point (on-screen) to co-ordinates (real world)
H_w2s = None #homography: co-ordinates (real world) to point (on-screen)

def load_corners():
    global screen_corners, world_corners
    screen_corners = params.screen_points
    world_corners = params.world_points

def calculate_homography(src_points, dst_points):
    src_points = np.array(src_points)
    dst_points = np.array(dst_points)
    A = []
    for i in range(4):
        x, y = src_points[i]
        x_prime, y_prime = dst_points[i]
        A.append([-x, -y, -1, 0, 0, 0, x * x_prime, y * x_prime, x_prime])
        A.append([0, 0, 0, -x, -y, -1, x * y_prime, y * y_prime, y_prime])

    A = np.array(A)

    _, _, V = np.linalg.svd(A)
    h = V[-1]
    H = h.reshape(3, 3)

    return H

def apply_homography(H, point):
    point = np.array([point[0], point[1], 1])
    transformed_point = H @ point
    transformed_point /= transformed_point[2]
    return transformed_point[:2]

def rearrange_points(points):
    pnts = [[],[],[],[]]
    mid = [
        (points[0][0]+points[1][0]+points[2][0]+points[3][0])/4,
        (points[0][1]+points[1][1]+points[2][1]+points[3][1])/4
    ]
    for p in points:
        if(p[0] < mid[0] and p[1] < mid[1]):
            pnts[0] = p
        elif(p[0] > mid[0] and p[1] < mid[1]):
            pnts[1] = p
        elif(p[0] > mid[0] and p[1] > mid[1]):
            pnts[2] = p
        elif(p[0] < mid[0] and p[1] > mid[1]):
            pnts[3] = p
    return pnts

def setKeyboardCornersOnScreen(corners):
    global screen_corners, H_k2d, H_s2w, H_w2s, detected_corners
    detected_corners = rearrange_points(corners)
    # detected_corners = screen_corners
    try:
        H_k2d = calculate_homography(keyb_corners, detected_corners)
        H_s2w = calculate_homography(screen_corners, world_corners)
        H_w2s = calculate_homography(world_corners, screen_corners)
    except:
        pass

def getScrPos(key):
    try:
        if(H_k2d is None):
            return None
        if key in key_pos:
            kPos = key_pos[key]
            sPos = apply_homography(H_k2d, kPos)
            return sPos
    except:
        pass
    return None

def getKeyPos(key):
    try:
        if(H_k2d is None or H_s2w is None):
            return None
        if key in key_pos:
            kPos = key_pos[key]
            sPos = apply_homography(H_k2d, kPos)
            wPos = apply_homography(H_s2w, sPos)
            return wPos
    except:
        pass
    return None

def getPfrS(sPos):
    return apply_homography(H_s2w, sPos)

def getPfrW(wPos):
    return apply_homography(H_w2s, wPos)

w_pos = getKeyPos("w")

print("Transformed Point:", w_pos)
