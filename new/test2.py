import math
import numpy as np
import mapping
def scan2pointcloud(scan):
    first = True
    points_cloud = np.array([])

    for point in scan:
        angle = point[0]
        dist = point[1]
        x_coord = dist*math.cos(angle)
        y_coord = dist*math.sin(angle)
        if first:
            points_cloud = np.array([x_coord, y_coord])
            first = False
        else:
            point = np.array([x_coord, y_coord])
            points_cloud = np.vstack((points_cloud, point))
    return points_cloud
if __name__ == '__main__':
    first = True
    points_cloud = np.array([])
    file = open("log.txt", "r")
    robot = {'time':[],'x':[],'y':[],'yaw':[],'scan':[]}
    for line in file:
        line = line.split(' ')
        robot['time'].append(float(line[0]))
        robot['x'].append(float(line[1]))
        robot['y'].append(float(line[2]))
        robot['yaw'].append(float(line[3]))
        scan = []
        for i, var in enumerate(line[3:]):
            angle = i*3.14/180
            dist = float(var)/1000
            scan.append([angle, dist])
        robot['scan'].append(scan)

    # load start pose
    startPose = np.matrix([[0],[0],[0]])


    """
    Parameter
    """

    # parameter mapping from octomap paper
    l_occupied = 0.85
    l_free = -0.4
    l_min = -2.0
    l_max = 3.5
    # parameter particle Filter
    stddPos = 0.1
    stddYaw = 0.20

    """
    Create empty GRID [m] and initialize it with 0, this Log-Odd 
    value is maximum uncertainty (P = 0.5)
    Use the ground truth trajectory to calculate width and length of the grid
    """
    resolution = 0.1

    grid = np.zeros((int(20.0/resolution),int(20.0/resolution)),order='C')

    """
    Calculate best startposition in grid
    """
    offset = np.array([10.0,10.0])

    """
    Other variables
    """
    # save estimated pose (x y yaw) in this list
    trajectory = []
    # save the estimated pose here (mouvement model)
    estimatePose = np.matrix([startPose[0,0],startPose[1,0],startPose[2,0]])



    trajectory = []

    # Позиция робота в системе координат ghj
    xPos = robot['x'][0]
    yPos = robot['y'][0]
    yaw  = robot['yaw'][0]
    
       
    pointcloud = scan2pointcloud(robot['scan'][0])
    # add measurement to grid
    x = pointcloud[:,0]
    y = pointcloud[:,1]
    mapping.addMeasurement(grid,x,y,estimatePose ,offset,resolution,l_occupied,l_free,l_min,l_max)
