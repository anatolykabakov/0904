import matplotlib.pyplot as plt
import math
import numpy as np
from astar import AStar
from lib import Serial

BASE_WIDTH = 248    # millimeters
WHEEL_RADIUS = 47
dt =0.1
log = open("log.txt", "w")
class Map(object):
    def __init__(self, width, height, map_size_x, map_size_y):
        self.i = []
        self.j = []
        self.width = width
        self.height = height
        self.block = np.zeros((map_size_x, map_size_y))
    def set_index(self, x, y):
        self.i.append(int(x*self.width))
        self.j.append(int(y*self.height))
        print(self.i)
        self.block[self.i][self.j] = -1
    def get_pos(self):
        x = self.width*self.i/2
        y = self.height*self.j/2
    def plot_map(self):
        return self.block
##        plt.show()

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

class Robot_Diff(object):
    def __init__(self, x, y, yaw, v, radius, length):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.radius = radius
        self.length = length
        
    def update(self, speed_right, speed_left, dt):
        self.yaw += dt * (speed_right - speed_left)/self.length
        self.yaw  = normalize_angle(self.yaw)
        self.x   += dt * 0.5 * (speed_right + speed_left)*math.cos(self.yaw)
        self.y   += dt * 0.5 * (speed_right + speed_left)*math.sin(self.yaw)
        
    def getPos(self):
        return self.x, self.y, self.yaw

class Controller(object):
    def __init__(self, robot, linear_velocity, angular_velocity, error_pos):
        self.robot = robot
        self.linear_velocity  = linear_velocity
        self.angular_velocity = angular_velocity
        self.error_pos = error_pos
        self.speed_right = 0
        self.speed_left = 0
        self.error_orientation = 0
        self.error_pos = 0

    def orientation_controller(self, x_goal, y_goal, x, y, theta):
        x_diff = x_goal - x
        y_diff = y_goal - y
        self.error_pos = math.sqrt(x_diff**2 + y_diff**2)
        self.error_orientation = normalize_angle(math.atan2(y_diff, x_diff) - theta)
##        print('goal: ' + str(math.atan2(y_diff, x_diff))+' - theta:'+str(theta)+' = '+str(self.error_orientation))

    def speed_control(self, target_speed, current_speed, delta_time, speed_up):
        self.angular_velocity = self.error_orientation/delta_time
        if speed_up:
            self.linear_velocity = (target_speed - self.robot.v)
        else:
            self.linear_velocity = (self.error_pos)
            
        self.restrictions_speed(target_speed, 0.3)
        self.control_move()
        
        self.speed_right = ((2*self.linear_velocity)+(self.angular_velocity*self.robot.length))/2# m/sec
        self.speed_left  = ((2*self.linear_velocity)-(self.angular_velocity*self.robot.length))/2
        
    def restrictions_speed(self, max_linear_speed, max_angular_speed):
        if self.linear_velocity > max_linear_speed:
            self.linear_velocity = max_linear_speed
        if self.linear_velocity <  -max_linear_speed:
            self.linear_velocity = -max_linear_speed
        if self.angular_velocity > max_angular_speed:
            self.angular_velocity = max_angular_speed
        if self.angular_velocity <  -max_angular_speed:
            self.angular_velocity = -max_angular_speed
            
    def control_move(self):
        if abs(self.error_orientation) > 0.1:
            self.linear_velocity = 0

            
def read_log(filename, log):
    f = open(filename, "r")
    for line in f:
        line = line.replace('\n', '')
        f = line.split(' ')
        robot['time'].append(float(f[0]))
        robot['x'].append(4+float(f[1]))
        robot['y'].append(4+float(f[2]))
        robot['yaw'].append(normalize_angle(float(f[3])))
        begin = line.index('[')
        scan = line[begin:].replace('\n','')
        scan = scan[2:-2].split('], [')
        state = [4+float(f[1]), 4+float(f[2]), float(f[3])]
        scan_data = []
        for i in scan:
            s = i.split(', ')
            if float(s[1])/1000 < 4:
                meas = [float(s[1])/1000, math.radians(float(s[0]))]
                scan_data.append(meas)
        robot['scan'].append(scan_data) 

def tracking_control(trajectory):
    if trajectory[len(trajectory)-1] != trajectory[i]:
            if controller.error_pos>0.05:
                point = trajectory[i]
                print(point)
                x_goal = point[0]
                y_goal = point[1]
                speed_up = True
            else:
                i+=1
                controller.error_pos=1
                point = trajectory[i]
                print(point)
                x_goal = point[0]
                y_goal = point[1]
                speed_up = True
    else:
        speed_up = False


if __name__ == '__main__':
    serial   = Serial('com3', 57600)
    robot= {'time':[],'x':[], 'y':[], 'yaw':[], 'scan': []}
    read_log("log2.txt", robot)
    m = Map(width=0.1, height=0.1, map_size_x=10, map_size_y=10)
    a = AStar()
    walls = ((0, 5), (1, 0), (1, 1), (1, 5), (2, 3),\
             (3, 1), (3, 2), (3, 5), (4, 1), (4, 4), (5, 1))
    a.init_grid(6, 6, walls, (0, 0), (5, 5))
    path = a.solve()
    robot = Robot_Diff(x=0, y=0, yaw=0, v=0, radius=WHEEL_RADIUS/1000, length=BASE_WIDTH/1000)
    controller = Controller(robot, linear_velocity=0, angular_velocity=0, error_pos=0)
    x = []
    y = []
    linear_vel = []
    angular_vel = []
    linear_vel.append(controller.linear_velocity)
    angular_vel.append(controller.angular_velocity)
    x_goal = 1
    y_goal = 1
    target_speed = 0.15
    sim_time = 50
    time = 0
    trajectory = [[1, 0], [1, 1], [0, 1], [0,0]]
    end_point = trajectory[-1]
    point = 0
    i=0
    controller.error_pos=1
    end_point =False
##    end_point = path[-1]
    while True:
##        if trajectory[len(trajectory)-1] != trajectory[i]:
##            if controller.error_pos>0.1:
##                point = trajectory[i]
##                print(point)
##                x_goal = point[0]
##                y_goal = point[1]
##                speed_up = True
##            else:
##                i+=1
##                controller.error_pos=1
##                point = trajectory[i]
##                print(point)
##                x_goal = point[0]
##                y_goal = point[1]
##                speed_up = True
##        else:
##            speed_up = False

##                log.close()

##        if end_point:
##            log.close()
##        if end_point != point:
##            speed_up = True
##        else:
##            speed_up = False
        
##        robot.update(controller.speed_right, controller.speed_left, dt)
        robot.v, robot.yaw, robot.x, robot.y = serial.getSerialData()
        robot.yaw = normalize_angle(robot.yaw)

        controller.orientation_controller(x_goal, y_goal, robot.x, robot.y, robot.yaw)
        controller.speed_control(target_speed, robot.v, dt, speed_up)
        serial.setSerialData(controller.linear_velocity, controller.angular_velocity)

        log.write(str(robot.x))
        x.append(robot.x)
        y.append(robot.y)
        linear_vel.append(controller.linear_velocity)
        angular_vel.append(controller.angular_velocity)
        plt.subplot(221)
        plt.plot(x, y, "-b")
        plt.subplot(222)
        plt.plot(linear_vel, "-g")
        plt.plot(angular_vel, "-r")
        plt.pause(0.001)
    log.close()

##    for point in trajectory:
##        controller.error_pos = 1
##        x_goal = point[0]
##        y_goal = point[1]
##        if end_point != point:
##            speed_up = True
##        else:
##            speed_up = False
##        while controller.error_pos>0.05:
##            robot.update(controller.speed_right, controller.speed_left, dt)
##            controller.orientation_controller(x_goal, y_goal, robot.x, robot.y, robot.yaw)
##            controller.speed_control(target_speed, robot.v, dt, speed_up)
##            x.append(robot.x)
##            y.append(robot.y)
##            linear_vel.append(controller.linear_velocity)
##            angular_vel.append(controller.angular_velocity)
##            plt.subplot(221)
##            plt.plot(x, y, "-b")
##            plt.subplot(222)
##            plt.plot(linear_vel, "-g")
##            plt.plot(angular_vel, "-r")
##            plt.pause(0.001)
    
    
