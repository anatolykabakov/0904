import time
import math
from robot import Robot
import numpy as np
from numpy.random import uniform
LIDAR_DEVICE            = '/dev/ttyUSB0'
ARDUINO_HCR             = '/dev/ttyACM0'

def create_uniform_particles(x_range, y_range, hdg_range, N):
    cx = []
    cy = []
    cx = uniform(x_range[0], x_range[1], size=N)
    cy = uniform(y_range[0], y_range[1], size=N)
    return cx, cy

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

def potencial_field(xr, yr, xg, yg, Vu, scan):
    '''
    xr - x координата робота в метрах
    yr - y координата робота в метрах
    xg - x координата цели в метрах
    yg - y координата цели в метрах
    Vu - желаемая скорость движения
    scan - массив точек лидара
    '''
    rd = 2 # дистанция рекции
    Ka = 1.0 # коэффициент притяжения
    Kr = 2.0 # коэффициент отталкивания
    dg = math.sqrt((xg-xr)**2+(yg-yr)**2)#расстояние до цели
    if dg <= 0.1:
        stop = True
    else:
        stop = False
    Xrf = []
    Yrf = []
    test_dist = []

    #Цикл по всем измерениям лидара
    for point in scan:
        alpha = point[0] #угол измерения лидара градусы
        dist  = point[1] #дистанция измерения лидара миллиметры
        alpha = alpha*math.pi/180 #градусы в радианты
        #alpha = alpha - 2*math.pi
        dist  = dist/1000 # миллиметры в метры
        
        if (0<=alpha) and (alpha<=math.pi/2):
            gamma = -alpha
        if (3*math.pi/2 <= alpha) and (alpha<= 2*math.pi):
            gamma = 2*math.pi- alpha
        per = -1*math.pi/2
        # Множество отталкивания
        if ((dist <= rd) and  ((0<=alpha<=math.pi/2) or (3*math.pi/2<=alpha<=2*math.pi)) ):
            f = ((rd - dist)/rd)**2
            Xrf.append(f*math.cos(gamma))
            Yrf.append(f*math.sin(gamma))
            test_dist.append(round(dist,1))
        
    #Общий вектор отталкивания
    #for dist in test_dist:
    print(test_dist)
    if len(Xrf)>0:
        RF = [sum(Xrf)/len(Xrf), sum(Yrf)/len(Yrf)]
    else:
        RF=[0,0]
    print(RF)
    
    #Вектор притяжения
    Xra = (xg-xr)/dg
    Yra = (yg-yr)/dg
    RA = [Xra,Yra]
    print(RA)
    #Вектор движения
    Xrf = RF[0]
    Yrf = RF[1]
    RV_x = Xra*Ka-Xrf*Kr
    RV_y = Yra*Ka-Yrf*Kr
    print(RV_x, RV_y)
    LinearVelocity = Vu*math.sqrt(RV_x**2+RV_y**2)
    #AngularVelocity = Vu*2*math.atan2(RV_y, RV_x)/0.1875/math.pi
    AngularVelocity = 3*math.atan2(RV_y, RV_x)
    return LinearVelocity, AngularVelocity,stop
        

def scan2distVec(scan):
    scanSize = 360
    distVec = [0 for i in range(scanSize)]

    for point in scan: # create breezySLAM-compatible data from raw scan data
      dist = point[1]
      index = int(point[0])
      if not 0 <= index < scanSize: continue
      distVec[index] = int(dist)
    return distVec

if __name__ == '__main__':
    
    robot = Robot(0.0682, 0.275, ARDUINO_HCR, LIDAR_DEVICE)
    '0. Randomly generate a bunch of particles'
    N = 10

    x_range = (0,2)
    y_range = (0,2)
    hdg_range = (0, 6.28)
    cx, cy = create_uniform_particles(x_range, y_range, hdg_range, N)

    Vu = 0.3
##    xg, yg = 30, 0
    target_idx = 0
    LinearVelocity = 0.2
    AngularVelocity = 0

    stop = False
    while True:
        try:
            
            robot.sense()
            robot.update_state()
            xg, yg, target_idx = get_point(cx, cy, robot, target_idx)
            AngularVelocity = controller(robot, xg, yg)
            #LinearVelocity, AngularVelocity,stop = potencial_field(robot.x, robot.y, xg, yg, Vu, robot.scan)
            robot.drive(LinearVelocity, AngularVelocity)
            #robot.drive(0.2, 0.5)
            #robot.write_log()
            #print('vr: {0}, vl: {1}'.format(robot.vr, robot.vl))
        except KeyboardInterrupt:
            robot.stop()
