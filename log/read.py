import matplotlib.pyplot as plt
import math
import numpy as np
# import bresenham algorithm
from lib import bresenham

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

def polar2decart(scan):
    points_cloud = np.array([])
    first = True
    for point in scan:
        if first:
            first = False
            angle = 2*math.pi - point[0]
            dist  = point[1]
            x_coord = dist*math.cos(angle)
            y_coord = dist*math.sin(angle)
            points_cloud = np.array([x_coord, y_coord])
        else:
            angle = point[0]
            dist  = point[1]
            x_coord = dist*math.cos(angle)
            y_coord = dist*math.sin(angle)
            point = np.array([x_coord, y_coord])
            points_cloud = np.vstack((points_cloud, point))
    return points_cloud
def read_data(filename, robot):
    file = open("log_03_05.txt", "r")
    
    for line in file:
        line = line.split(' ')
        robot['timestamp'].append(float(line[0]))
        robot['vel'].append(float(line[1]))
        robot['yaw'].append(float(line[2]))
        robot['x'].append(float(line[3]))
        robot['y'].append(float(line[4]))
        scan = []
        for i, var in enumerate(line[4:]):
            angle = i*math.pi/180
            dist  = float(var)/1000
            scan.append([angle, dist])
        robot['scan'].append(scan)
    return robot

def dead_rec_model(x, u, DT):
        #DT = 0.1
        F = np.array([[1.0, 0, 0],\
                      [0, 1.0, 0],\
                      [0, 0, 1.0]])
        B = np.array([[DT * math.cos(x[2, 0]), 0],\
                      [DT * math.sin(x[2, 0]), 0],\
                      [0.0, DT]])
        x = (F @ x) + (B @ u)
        return x


if __name__ == '__main__':
    xDR = np.array([[0, 0, 0]]).T
    '//------------------- read log----------------------'
    robot = {'timestamp':[],\
             'x':[],\
             'y':[],\
             'yaw':[],\
             'vel':[],\
             'scan':[]}
    filename = "log_03_05.txt"
    robot = read_data(filename, robot)
    '//------------------- read log----------------------'
     
    # resolution of the grid
    resolution = 0.1
    l_occupied = 0.85
    l_free = -0.4

    l_max = 3.5
    l_min = -2.0
    # create grid 
    grid = np.zeros((int(20.0/resolution),int(20.0/resolution)),order='C')

    xPos_log=[]
    yPos_log=[]
##    for i in range(len(robot['x'])):
##        if i == 0:
##            continue
##        pointcloud = polar2decart(robot['scan'][i])
##        # Позиция робота в системе координат ghj
##        timestamp = robot['timestamp'][i]
##        xPos = robot['x'][i]
##        yPos = robot['y'][i]
##        #yaw = robot['yaw'][i]
##        time = robot['timestamp'][i] - robot['timestamp'][i-1]
##        u = np.array([[0.3, 0.5]]).T
##        xDR = dead_rec_model(xDR, u, time)
##        yaw = xDR[2,0]
##        # Матрица поворота 
##        R = np.matrix([[np.cos(yaw),np.sin(yaw)], [-np.sin(yaw),np.cos(yaw)]])
##        # Поворот массива координат лидара на угол робота
##        pointcloud = pointcloud * np.transpose(R)
##        #pointcloud = pointcloud + np.matrix([xPos, yPos])
##        xPos_log.append(xPos)
##        yPos_log.append(yPos)
##        plt.plot(xPos_log, yPos_log, 'or')
##        plt.plot(pointcloud[:,0],pointcloud[:,1],'.g')
##        plt.pause(0.001)
##        plt.clf()
    
    for i in range(len(robot['x'])):
        if i == 0:
            continue
        pointcloud = polar2decart(robot['scan'][i])
##        # Позиция робота в системе координат ghj
        xPos = robot['x'][i]
        yPos = robot['y'][i]
        #yaw = robot['yaw'][i]
        time = robot['timestamp'][i] - robot['timestamp'][i-1]
        u = np.array([[0.3, 0.5]]).T
        xDR = dead_rec_model(xDR, u, time)
        yaw = xDR[2,0]
        #xPos = xDR[0,0]
        #yPos = xDR[1,0]
        # Матрица поворота 
        R = np.matrix([[np.cos(yaw),np.sin(yaw)], [-np.sin(yaw),np.cos(yaw)]])
        # Поворот массива координат лидара на угол робота
        pointcloud = pointcloud * np.transpose(R)
        pointcloud = pointcloud - np.matrix([xPos, yPos])
        
        # Смещение координат локальной системы в глобальную систему координат 
        offset = np.array([10.0,10.0])

        for ii in range(pointcloud.shape[0]):

            if abs(pointcloud[ii,0]-xPos) <= 0.05 or abs(pointcloud[ii,1]-yPos) <= 0.05:#если значение 0, значит препятствие не обнаружено
                xi = math.ceil( (pointcloud[ii,0]+offset[0]) / resolution ) 
                yi = math.ceil( (pointcloud[ii,1]+offset[1]) / resolution )
                #print('zero')
            else:
                # round points to cells
                # Массив координат лидара в системе глобальных координат / размер ячейки = массив лидара
                # в системе координат карты 
                xi = math.ceil( (pointcloud[ii,0]+offset[0]) / resolution ) 
                yi = math.ceil( (pointcloud[ii,1]+offset[1]) / resolution )
                # set beam endpoint-cells as occupied 
                if xi >= grid.shape[0] or yi >= grid.shape[1]:
                        continue
                grid[xi,yi] += l_occupied
                # value > threshold? -> clamping 
                if grid[xi,yi] > l_max:
                    grid[xi,yi] = l_max

            x_start = math.ceil((offset[0]+xPos)/resolution)
            y_start = math.ceil((offset[1]+yPos)/resolution)
            
            # calculate cells between sensor and endpoint as free with bresenham
            startPos = np.array([[x_start,y_start]])
            endPos = np.array([[xi,yi]])
            bresenhamPath = bresenham.bresenham2D(startPos, endPos)
            
            # set free cells as free
            for jj in range(bresenhamPath.shape[0]):
                path_x = int(bresenhamPath[jj,0])
                path_y = int(bresenhamPath[jj,1])
                
                grid[path_x, path_y] += l_free
                
                # value < threshold? -> clamping
                if grid[path_x, path_y] < l_min:
                    grid[path_x, path_y] = l_min

        plt.figure(3)
        plt.clf()
        plt.imshow(grid[:,:].T, interpolation ='none', cmap = 'binary', origin='lower')
        plt.scatter([(10+pointcloud[:,0])/0.1],[(10+pointcloud[:,1])/0.1],c='m',s=10,edgecolors='none')
        plt.pause(0.001)


