import numpy as np
import math

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

def pointscloud2scan(robot, points_cloud):
    'convert points x, y to distance and angle '
    scan = []
    for point in points_cloud:
        point_x = point[0]
        point_y = point[1]
        x_diff = point_x# - robot.gtPose.x
        y_diff = point_y# - robot.gtPose.y

        distance = math.sqrt(x_diff**2 + y_diff**2)
        if x_diff != 0:
            angle    = math.atan(y_diff/ x_diff) 
        else:
            angle    = math.atan(0) 
        scan.append([distance, normalize_angle(angle)])

    return scan

def scan2distVec(points):
    distVec = []
    for point in points: # create breezySLAM-compatible data from raw scan data
        X = round(point[1],2)
        Y = round(point[0],2)
        distVec.append(X)
        distVec.append(Y)
    return distVec

def dist2d(point1, point2):
    """
    Euclidean distance between two points
    :param point1:
    :param point2:
    :return:
    """

    x1, y1 = point1[0:2]
    x2, y2 = point2[0:2]

    dist2 = (x1 - x2)**2 + (y1 - y2)**2

    return math.sqrt(dist2)

def cart2pol(x, y):
    rho = np.sqrt(x**2 + y**2)
    phi = np.arctan2(y, x)
    return(rho, phi)

def pol2cart(rho, phi):
    x = rho * np.cos(phi)
    y = rho * np.sin(phi)
    return(x, y)

def pointcloud2scan(robot, pointcloud):
    scan = []
    xr = robot.EstPose.x
    yr = robot.EstPose.y
    for ii in range(pointcloud.shape[0]):
        x_coord = pointcloud[ii,0]
        y_coord = pointcloud[ii,1]
        deltaX = x_coord# - xr
        deltaY = y_coord# - yr
        rho, phi = cart2pol(deltaX, deltaY)
        scan.append([rho, phi])
    return scan


# Find the derivative in scan data, ignoring invalid measurements.
def compute_derivative(scan, min_dist):
    distance = []
    for measurement in scan:
        distance.append(measurement[0])
    jumps = [ 0 ]
    for i in range(1, len(distance) - 1):
        l = distance[i-1]
        r = distance[i+1]
        if l > min_dist and r > min_dist:
            derivative = (r - l) / 2.0
            jumps.append(derivative)
        else:
            jumps.append(0)
    jumps.append(0)  
    return jumps




def find_cylinders(scan, scan_derivative, jump, min_dist):
    distance = []
    angle = []
    for measurement in scan:
        distance.append(measurement[0])
        angle.append(measurement[1])
    cylinder_list = []
    on_cylinder = False
    sum_ray, sum_depth, rays, sum_angle = 0.0, 0.0, 0.0, 0.0
    #mass_dist = []
    #min_dist = 0

    for i in range(len(scan_derivative)):
        if scan_derivative[i] < -jump and distance[i] > min_dist:
            sum_ray, sum_depth, rays, sum_angle = 0.0, 0.0, 0.0, 0.0
            #mass_dist = []
            #min_dist = 0
            on_cylinder = True

        if on_cylinder is True:
            sum_ray += i
            sum_depth += distance[i]
            sum_angle += angle[i]
            rays += 1
            #mass_dist.append(distance[i])

            if scan_derivative[i] > jump and distance[i] > min_dist and rays > 5:
                on_cylinder = False
                average_depth = sum_depth / rays
                average_angle = sum_angle / rays
                average_ray = int(sum_ray / rays)
                #min_dist = min(mass_dist)
                #print(min_dist)

                #cylinder_list.append((average_ray, distance[average_ray], angle[average_ray]))
                cylinder_list.append((average_ray, average_depth, average_angle))
                #cylinder_list.append((average_ray, min_dist, angle[average_ray]))

    return cylinder_list

def cylinders2decart(robot, cylinder_list):
    coord = []
    xr = robot.gtPose.x
    yr = robot.gtPose.y
    angle_r = robot.gtPose.orientation
    for cylinder in cylinder_list:
        dist = cylinder[1]
        angle = cylinder[2]
        x_coord = xr + dist*math.cos(angle_r + angle)
        y_coord = yr + dist*math.sin(angle_r + angle)
        #x_coord = dist*math.cos(angle)
        #y_coord = dist*math.sin(angle)
        coord.append([x_coord, y_coord])
    return coord

def prepare_kalman(robot):
    # add noise to gps x-y
    z = np.zeros((0, 3))
    for i in range(len(robot.cylinders)):
        dist = robot.cylinders[i][1]
        angle = robot.cylinders[i][2]
        
        zi = np.array([dist, angle, i])
        z = np.vstack((z, zi))
    return z