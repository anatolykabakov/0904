import numpy as np
import math
from Utils import pointcloud2scan, cylinders2decart, normalize_angle

class Landmarks(object):
    def __init__(self):
        self.min_dist = 0.05
        self.max_dist = 2
        self.depth_jump = 0.5

        self.scan = []
        self.derivate = []
        self.cylinders = []
        self.landmark = []
        self.lm = np.array([])

    def extract_landmarks(self, scan):
        if len(scan):
            self.derivate = self.compute_derivative(scan, self.min_dist)
            self.cylinders = self.find_cylinders(scan, self.derivate, self.depth_jump, self.min_dist, self.max_dist)
            self.lm = self.to_numpy()
        return self.lm


    def compute_derivative(self, scan, min_dist):
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

    def find_cylinders(self, scan, scan_derivative, jump, min_dist, max_dist):
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

    def to_numpy(self):
            # add noise to gps x-y
            z = np.zeros((0, 3))
            for i in range(len(self.cylinders)):
                dist = self.cylinders[i][1]
                angle = self.cylinders[i][2]
                
                zi = np.array([dist, angle, i])
                z = np.vstack((z, zi))
            return z
    def getLandmarks(self):
        return np.array([[-2, 4], [2, 4], [0, -2]])
