import numpy as np
from numpy.random import uniform
import matplotlib.pyplot as plt
import math

class Robot(object):

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        """Instantiate the object."""
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v

    def update(self, v, w, dt):
        self.v = v
        self.yaw += w * dt
        self.yaw = normalize_angle(self.yaw)
        self.x += self.v * np.cos(self.yaw) * dt
        self.y += self.v * np.sin(self.yaw) * dt
        

def plot_particles_f(particles):
    plt.scatter(particles[:, 0], particles[:, 1], color='k', marker=',', s=1)
    plt.xlabel('ось x')
    plt.ylabel('ось y')
    plt.pause(0.1)


def create_uniform_particles(x_range, y_range, hdg_range, N):
##    particles = np.empty((N, 2))
##    particles[:, 0] = uniform(x_range[0], x_range[1], size=N)
##    particles[:, 1] = uniform(y_range[0], y_range[1], size=N)
    cx = []
    cy = []
    cx = uniform(x_range[0], x_range[1], size=N)
    cy = uniform(y_range[0], y_range[1], size=N)

    return cx, cy

def dead_rec_model(x, u):
    DT = 0.1
    F = np.array([[1.0, 0, 0],\
                  [0, 1.0, 0],\
                  [0, 0, 1.0]])
    B = np.array([[DT * math.cos(x[2, 0]), 0],\
                  [DT * math.sin(x[2, 0]), 0],\
                  [0.0, DT]])
    x = (F @ x) + (B @ u)
    
    return x

def get_point(cx, cy, robot, target_idx):
    if target_idx >= len(cx):
        target_idx = 0
        print('index 0')
    dx = cx[target_idx]-robot.x
    dy = cy[target_idx]-robot.y
    d = math.sqrt(dx**2+dy**2)
    if d < 0.2:
        target_idx += 1
    if target_idx >= len(cx):
        target_idx = 0
        print('index 0')
    xg = cx[target_idx]
    yg = cy[target_idx]
    return xg, yg, target_idx

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

def controller(robot, xg, yg):
    dx = xg-robot.x
    dy = yg-robot.y
    e_angle = normalize_angle(math.atan2(dy, dx)-robot.yaw)
    return e_angle

if __name__ == '__main__':

    robot = Robot()
    '0. Randomly generate a bunch of particles'
    N = 10

    x_range = (0,2)
    y_range = (0,2)
    hdg_range = (0, 6.28)
    cx, cy = create_uniform_particles(x_range, y_range, hdg_range, N)

    plt.plot(cx, cy, '.b')
    plt.pause(0.001)
    xDR = np.array([[0, 0, 0]]).T
    log_xDR_x = []
    log_xDR_y = []
    linear_velocity = 1#m/s
    angular_velocity = 0.5#rad/s


    target_idx = 0
    v = 0.2

    while True:
        xg, yg, target_idx = get_point(cx, cy, robot, target_idx)
        #print(xg, yg)
        w = controller(robot, xg, yg)
        #print(w)
        robot.update(v, w, 0.1)

        log_xDR_x.append(robot.x)
        log_xDR_y.append(robot.y)
        plt.plot(log_xDR_x, log_xDR_y, '.b')
        plt.pause(0.001)
        plt.clf()


