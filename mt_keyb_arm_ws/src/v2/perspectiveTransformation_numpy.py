import numpy as np

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

# Set the corners of the source rectangle and the transformed rectangle
src_points = [[123, 254], [533, 212], [694, 706], [173, 804]]  # Source rectangle
dst_points = [[0, 0], [80, 0], [80, 100], [0, 100]]  # Transformed rectangle


def apply_homography(H, point):
    point = np.array([point[0], point[1], 1])
    transformed_point = H @ point
    transformed_point /= transformed_point[2]
    return transformed_point[:2]

# Get the homography matrix
H = calculate_homography(src_points, dst_points)

# Apply the homography to a point
point = [374, 478]  # Point in the source rectangle
transformed_point = apply_homography(H, point)

print("Transformed Point:", transformed_point)
