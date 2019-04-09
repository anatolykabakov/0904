import matplotlib.pyplot as plt

from robot import Robot_Diff
from controller import Controller
from tracking import MotionControl
from gridmap import OccupancyGridMap
from manager import TaskManager
from utils import normalize_angle
from lib import Serial
import numpy as np
import time
#from rplidar import RPLidar
##lidar = RPLidar('/dev/ttyUSB0')

from lib import Serial

BASE_WIDTH = 248    # millimeters
WHEEL_RADIUS = 47
file = open('log_vel_nopid.txt', 'w')
def controlSystem(controller, tracking, manager):
    if manager.run == True :
        if manager.task == 'move2point':
            tracking.move_to_point(tracking.x_goal, tracking.y_goal)
        if manager.task == 'move2trajectory':
            tracking.move_to_trajectory(tracking.trajectory)
        controller.update()

if __name__ == '__main__':
    serial   = Serial('com4', 57600)
    robot = Robot_Diff(x=5, y=5, yaw=0, v=0, radius=WHEEL_RADIUS/1000, length=BASE_WIDTH/1000, dt=0.1)
    controller = Controller(robot, linear_velocity=0, angular_velocity=0, target_speed = 0.2, max_angular_velocity =1)
    tracking = MotionControl(robot, controller)
    grid_map = np.zeros((100,100))
    gridmap = OccupancyGridMap(grid_map, cell_size=0.1)
    manager = TaskManager(tracking, controller, gridmap)
    x = []
    y = []
    linear_vel = []
    angular_vel = []
    rob_vel = []
    rob_vel.append(0)
    linear_vel.append(controller.linear_velocity)
    angular_vel.append(controller.angular_velocity)
    manager.cnc_command = 'move2trajectory 1,0 1,1 0,1 0,0'
##    manager.cnc_command = 'move2point 1 0'
##    manager.cnc_command = 'movevelocity 0.3,1.0'
##    trajectory = [[1, 0], [1, 1], [0, 1], [0,0]]
    prev_time = 0
    current_time = 0
    delta_time = 0
    c_time  = 0
    c_time_mass = []
    c_time_mass.append(c_time)
    current_time = time.time()
    rightVel = []
    leftVel = []
    rightVel.append(0)
    leftVel.append(0)
    while True:

##
        robot.v, robot.yaw, robot.x, robot.y = serial.getSerialData()
##        robot.v, rightVelocity, leftVelocity,  robot.yaw, robot.x, robot.y = serial.getSerialData()
##        print(str(robot.yaw))
##        robot.yaw = normalize_angle(robot.yaw)
        controlSystem(controller, tracking, manager)
        serial.setSerialData(controller.linear_velocity, controller.angular_velocity)
##        robot.update(controller.linear_velocity, controller.angular_velocity)
##        gridmap.update(robot, scan)

        manager.update()
##        print(str(controller.speed_right)+' '+str(controller.speed_left))
        prev_time = current_time
        current_time = time.time()
        delta_time = current_time - prev_time
        c_time += delta_time

        x.append(robot.x)
        y.append(robot.y)
        
        rob_vel.append(robot.v)
        c_time_mass.append(c_time)
        linear_vel.append(controller.linear_velocity)
        angular_vel.append(controller.angular_velocity)
        plt.subplot(221)
        plt.plot(x, y, "-b")
        plt.subplot(222)
        plt.plot(c_time_mass, rob_vel, "-b")
        plt.plot(c_time_mass,linear_vel, "-g")
        plt.plot(c_time_mass,angular_vel, "-r")

        plt.pause(0.001)
        if manager.done == True:
            serial.setSerialData(0, 0)
            file.close()
            print('Done!')
