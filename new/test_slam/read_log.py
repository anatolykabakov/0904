import matplotlib.pyplot as plt
import numpy as np
import math
from ekf_slam import ekf_slam
from Landmarks import Landmarks
from Utils import normalize_angle




def motion_model(x, u, DT):

    F = np.array([[1.0, 0, 0],
                  [0, 1.0, 0],
                  [0, 0, 1.0]])

    B = np.array([[DT * math.cos(x[2, 0]), 0],
                  [DT * math.sin(x[2, 0]), 0],
                  [0.0, DT]])

    x = (F @ x) + (B @ u)
    return x

def cylinders2decart(xr, yr, angle_r, cylinder_list):
    coord = []

    for cylinder in cylinder_list:
        dist = cylinder[1]
        angle = cylinder[2]
        x_coord = xr + dist*math.cos(angle_r + angle)
        y_coord = yr + dist*math.sin(angle_r + angle)
        plt.plot(x_coord, y_coord, '.b')
        
    plt.pause(0.001)
    return coord

def plot_dist(scan, derivate, cylinders):
    plt.clf()
    distance = []
    for measurement in scan:
        distance.append(measurement[1])

    plt.plot(distance, '-b')
    plt.plot(derivate, '-b')
    plt.scatter([c[0] for c in cylinders], [c[1] for c in cylinders],c='r', s=200)
    plt.pause(0.01)

if __name__ == '__main__':
    first = True
    points_cloud = np.array([])
    file = open("log.txt", "r")
    robot = {'time':[],'vr':[], 'vl':[],'x':[],'y':[],'yaw':[],'scan':[]}
    for line in file:
        line = line.split(' ')
        robot['time'].append(float(line[0]))
        robot['vr'].append(float(line[1]))
        robot['vl'].append(float(line[2]))
        robot['x'].append(float(line[3]))
        robot['y'].append(float(line[4]))
        robot['yaw'].append(normalize_angle(float(line[5])))
        scan = []
        mass_dist = []
        mass_angle = []
        for i, var in enumerate(line[5:]):
            angle = normalize_angle(i*3.14/180 + float(line[5]))
            dist = float(var)/1000
            if dist != 0:
                mass_dist.append(dist)
                mass_angle.append(angle)
                scan.append([angle, dist])
        robot['scan'].append(scan)
        

    WHEELS_RAD = 0.0687
    WHEELS_DIST = 0.275
    landmarks = Landmarks()
    slam = ekf_slam(landmarks)
    xDR = np.zeros((3, 1))  # Dead reckoning
    for i in range(len(robot['time'])):
        if i==0:
            continue
        deltaTime  = robot['time'][i] - robot['time'][i-1]
        omegaRight = robot['vr'][i]/WHEELS_RAD
        omegaLeft  = robot['vl'][i]/WHEELS_RAD
        linear_velocity  = (WHEELS_RAD/2)*(omegaRight + omegaLeft)# meters per sec
        angular_velocity = (WHEELS_RAD/WHEELS_DIST)*(omegaRight - omegaLeft) #rad /s
        u = np.array([[linear_velocity, angular_velocity]]).T
        scan = robot['scan'][i]
        EstPose, var = slam.ekf_slam(u, scan, deltaTime)
        xDR = motion_model(xDR, u, deltaTime)
        #plt.plot(robot['x'][i], robot['y'][i],'.b')
        plt.plot(EstPose[0,0], EstPose[1,0],'.r')
        #plt.plot(xDR[0,0], xDR[1,0],'.g')
        cylinders2decart(EstPose[0,0], EstPose[1,0], EstPose[2,0], landmarks.cylinders)
        
        plt.pause(0.001)
        #plot_dist(scan, landmarks.derivate, landmarks.cylinders)
##        plt.clf()
##        plt.plot(mass_dist, '-b')
##        plt.pause(0.001)
    
