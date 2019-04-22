import matplotlib.pyplot as plt
import numpy as np
import math

def compute_derivative(scan, min_dist):
    distance = []
    for measurement in scan:
        distance.append(measurement[1])
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

def find_cylinders(scan, scan_derivative, jump, min_dist, max_dist):
    distance = []
    angle = []
    for measurement in scan:
        distance.append(measurement[1])
        angle.append(measurement[0])
    cylinder_list = []
    on_cylinder = False
    sum_ray, sum_depth, rays, sum_angle = 0.0, 0.0, 0.0, 0.0
    #mass_dist = []
    #min_dist = 0

    for i in range(len(scan_derivative)):
        if scan_derivative[i] < -jump and distance[i] > min_dist and  distance[i] < max_dist:
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

            if scan_derivative[i] > jump and distance[i] > min_dist and rays > 2 and rays < 7:
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
        robot['yaw'].append(float(line[5]))
        scan = []
        mass_dist = []
        mass_angle = []
        for i, var in enumerate(line[5:]):
            angle = i*3.14/180
            dist = float(var)/1000
            if dist != 0:
                mass_dist.append(dist)
                mass_angle.append(angle)
                scan.append([angle, dist])
        robot['scan'].append(scan)
        min_dist = 0.05
        max_dist = 2
        depth_jump = 0.5
        derivative = compute_derivative(scan, 0.05)
        cylinders = find_cylinders(scan, derivative, depth_jump, min_dist, max_dist)
        #plot_dist(scan, derivative, cylinders)


    for i in range(len(robot['time'])):
        plt.plot(robot['x'][i], robot['y'][i],'.g')
        plt.pause(0.001)
##        plt.clf()
##        plt.plot(mass_dist, '-b')
##        plt.pause(0.001)
    
