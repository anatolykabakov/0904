import numpy as np

def normalize_angle(angle):
    """
    Normalize an angle to [-pi, pi].

    :param angle: (float)
    :return: (float) Angle in radian in [-pi, pi]
    """
    while angle > np.pi:
        angle -= 2.0 * np.pi

    while angle < -np.pi:
        angle += 2.0 * np.pi

    return angle

def scan2distVec(scan):
    scanSize = 360
    distVec = [0 for i in range(scanSize)]

    for point in scan: # create breezySLAM-compatible data from raw scan data
      dist = point[1]
      index = int(point[0])
      if not 0 <= index < scanSize: continue
      distVec[index] = int(dist)
    return distVec
