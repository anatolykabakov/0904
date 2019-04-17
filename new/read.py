import matplotlib.pyplot as plt
import numpy as np
import math
import bresenham


class Particle(object):
    def __init__(self, x, y, yaw, weight):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.weight = weight

class GridSLAM(object):
    def __init__(self, cell_size=0.1):
        self.cell_size = cell_size
        # resolution of the grid
        self.resolution = cell_size
        self.l_occupied = 0.85
        self.l_free = -0.4
        self.l_max = 3.5
        self.l_min = -2.0
        self.offset = np.array([10.0,10.0])
        # create grid
        self.grid = np.zeros((int(20.0/self.resolution),int(20.0/self.resolution)),order='C')
        # save estimated pose (x y yaw) in this list
        self.trajectory = []
        self.startPose = np.matrix([[0],[0],[0]])
        # save the estimated pose here (mouvement model)
        self.estimatePose = np.matrix([0,0,0])
        self.first = True
        # standard deviation of position and yaw
        self.stddPos = 0.1
        self.stddYaw = 0.02
        # number of particels
        self.nrParticle = 500
        self.particles = []
        self.init_particles()
        self.bestParticles = []
        self.bestParticle = []

    def init_particles(self):
        for _ in range(0,self.nrParticle):
            x   = np.random.normal(self.startPose[0],self.stddPos)
            y   = np.random.normal(self.startPose[1],self.stddPos)
            yaw = np.random.normal(self.startPose[2],self.stddYaw)

            # create particle and append ist to list
            p = Particle(x,y,yaw,1)
            self.particles.append(p)

        # weight all particles
        for p in self.particles:
            R = np.matrix([[np.cos(p.yaw[0,0]),-np.sin(p.yaw[0,0])], [np.sin(p.yaw[0,0]),np.cos(p.yaw[0,0])]])
            pointcloudTransformed = pointcloud * np.transpose(R)
            
            # translate pointcloud
            pointcloudTransformed = pointcloudTransformed + np.matrix([p.x[0,0],p.y[0,0]])
            
            # weight particle
            p.weight = self.scan2mapDistance(pointcloudTransformed)
        # sort particles
        self.particles.sort(key = lambda Particle: Particle.weight,reverse=True)
        # get ten best particles
        self.bestParticles = self.particles[:10]

        # get best particle
        self.bestParticle = self.particles[0]
        #------------- RESAMPLE ----------------
        # standard deviation of position and yaw for resampling
        stddPosResample = self.stddPos#/1.0
        stddYawResample = self.stddYaw#/1.0

        # number of particels
        nrParticleResample = 50
        # delete old particles
        self.particles.clear()

        # create 50 particles for each of the best 10 particles
        for bp in self.bestParticles:
            for _ in range(0,nrParticleResample):
                x = np.random.normal(bp.x,stddPosResample)
                y = np.random.normal(bp.y,stddPosResample)
                yaw = np.random.normal(bp.yaw,stddYawResample)

                # create particle and append ist to list
                p = Particle(x,y,yaw,1)
                self.particles.append(p)
        #-----------WEIGHT------------------
        # weight all particles
        for p in self.particles:
            R = np.matrix([[np.cos(p.yaw[0,0]),-np.sin(p.yaw[0,0])], [np.sin(p.yaw[0,0]),np.cos(p.yaw[0,0])]])
            pointcloudTransformed = pointcloud * np.transpose(R)
            
            # translate pointcloud
            pointcloudTransformed = pointcloudTransformed + np.matrix([p.x[0,0],p.y[0,0]])
            
            # weight particle
            p.weight = self.scan2mapDistance(pointcloudTransformed)
        # sort particles
        self.particles.sort(key = lambda Particle: Particle.weight,reverse=True)
        # get ten best particles
        self.bestParticles = self.particles[:10]

        # get best particle
        self.bestParticle = self.particles[0]
        self.estimatePose = np.matrix([self.bestParticle.x[0,0],self.bestParticle.y[0,0],self.bestParticle.yaw[0,0]])
        # add position to trajectory
        self.trajectory.append(self.estimatePose)
    def fitscan2Map(self, pointcloud):
        # save the particles of the particle filter here
        self.particles = []
        for _ in range(0,self.nrParticle):
##            print(self.estimatePose[0,1])
            x   = np.random.normal(self.estimatePose[0,0],self.stddPos)
            y   = np.random.normal(self.estimatePose[0,1],self.stddPos)
            yaw = np.random.normal(self.estimatePose[0,2],self.stddYaw)

            # create particle and append ist to list
            p = Particle(x,y,yaw,1)
            self.particles.append(p)
        #-----------WEIGHT------------------
        # weight all particles
        for p in self.particles:
##            print(p.yaw)
            R = np.matrix([[np.cos(p.yaw),-np.sin(p.yaw)], [np.sin(p.yaw),np.cos(p.yaw)]])
            pointcloudTransformed = pointcloud * np.transpose(R)
            
            # translate pointcloud
            pointcloudTransformed = pointcloudTransformed + np.matrix([p.x,p.y])
            
            # weight particle
            p.weight = self.scan2mapDistance(pointcloudTransformed)
        # sort particles
        self.particles.sort(key = lambda Particle: Particle.weight,reverse=True)
        # get ten best particles
        self.bestParticles = self.particles[:10]

        # get best particle
        self.bestParticle = self.particles[0]
        #------------- RESAMPLE ----------------
        # standard deviation of position and yaw for resampling
        stddPosResample = self.stddPos#/1.0
        stddYawResample = self.stddYaw#/1.0

        # number of particels
        nrParticleResample = 50
        # delete old particles
        self.particles.clear()

        # create 50 particles for each of the best 10 particles
        for bp in self.bestParticles:
            for _ in range(0,nrParticleResample):
                x = np.random.normal(bp.x,stddPosResample)
                y = np.random.normal(bp.y,stddPosResample)
                yaw = np.random.normal(bp.yaw,stddYawResample)

                # create particle and append ist to list
                p = Particle(x,y,yaw,1)
                self.particles.append(p)
        #-----------WEIGHT------------------
        # weight all particles
        for p in self.particles:
            R = np.matrix([[np.cos(p.yaw),-np.sin(p.yaw)], [np.sin(p.yaw),np.cos(p.yaw)]])
            pointcloudTransformed = pointcloud * np.transpose(R)
            
            # translate pointcloud
            pointcloudTransformed = pointcloudTransformed + np.matrix([p.x,p.y])
            
            # weight particle
            p.weight = self.scan2mapDistance(pointcloudTransformed)
        # sort particles
        self.particles.sort(key = lambda Particle: Particle.weight,reverse=True)
        # get ten best particles
        self.bestParticles = self.particles[:10]

        # get best particle
        self.bestParticle = self.particles[0]
        #------------- RESAMPLE ----------------
        # standard deviation of position and yaw for resampling
        stddPosResample = self.stddPos#/1.0
        stddYawResample = self.stddYaw#/1.0

        # number of particels
        nrParticleResample = 50
        # delete old particles
        self.particles.clear()

        # create 50 particles for each of the best 10 particles
        for bp in self.bestParticles:
            for _ in range(0,nrParticleResample):
                x = np.random.normal(bp.x,stddPosResample)
                y = np.random.normal(bp.y,stddPosResample)
                yaw = np.random.normal(bp.yaw,stddYawResample)

                # create particle and append ist to list
                p = Particle(x,y,yaw,1)
                self.particles.append(p)
        #-----------WEIGHT------------------
        # weight all particles
        for p in self.particles:
            R = np.matrix([[np.cos(p.yaw),-np.sin(p.yaw)], [np.sin(p.yaw),np.cos(p.yaw)]])
            pointcloudTransformed = pointcloud * np.transpose(R)
            
            # translate pointcloud
            pointcloudTransformed = pointcloudTransformed + np.matrix([p.x,p.y])
            
            # weight particle
            p.weight = self.scan2mapDistance(pointcloudTransformed)
        # sort particles
        self.particles.sort(key = lambda Particle: Particle.weight,reverse=True)
        # get ten best particles
        self.bestParticles = self.particles[:10]

        # get best particle
        self.bestParticle = self.particles[0]
        self.estimatePose = np.matrix([self.bestParticle.x,self.bestParticle.y,self.bestParticle.yaw])
        # add position to trajectory
        self.trajectory.append(self.estimatePose)


    def scan2mapDistance(self, pcl):
        distance = 0;
        for i in range(pcl.shape[0]):
            # round points to cells
            xi = math.ceil( (pcl[i,0]+self.offset[0]) / self.resolution )
            yi = math.ceil( (pcl[i,1]+self.offset[0]) / self.resolution )
            if xi >= 200 or yi>=200:
                distance = 0
            else:
                distance += self.grid[xi,yi]
        return distance  

    def calc_weight(self, pointcloud):
        # weight all particles
        for p in self.particles:
            # rotate pointcloud
##            print(p.yaw[0,0])
            R = np.matrix([[np.cos(p.yaw[0,0]),np.sin(p.yaw[0,0])], [-np.sin(p.yaw[0,0]),np.cos(p.yaw[0,0])]])
            pointcloudTransformed = pointcloud * np.transpose(R)
            
            # translate pointcloud
            pointcloudTransformed = pointcloudTransformed + np.matrix([p.x[0,0],p.y[0,0]])
            
            # weight particle
            p.weight = self.scan2mapDistance(pointcloudTransformed)
        # sort particles
        self.particles.sort(key = lambda Particle: Particle.weight,reverse=True)

    def Resample(self):
        #------------- RESAMPLE ----------------
        # standard deviation of position and yaw for resampling
        stddPosResample = self.stddPos/5.0
        stddYawResample = self.stddYaw/5.0

        # number of particels
        nrParticleResample = 50
        # delete old particles
        self.particles.clear()

        # create 50 particles for each of the best 10 particles
        for bp in self.bestParticles:
            for _ in range(0,nrParticleResample):
                x = np.random.normal(bp.x,stddPosResample)
                y = np.random.normal(bp.y,stddPosResample)
                yaw = np.random.normal(bp.yaw,stddYawResample)

                # create particle and append ist to list
                p = Particle(x,y,yaw,1)
                self.particles.append(p)
        

    def Estimate(self):
        # get ten best particles
        self.bestParticles = self.particles[:10]
        # get best particle
        self.bestParticle = self.particles[0]
        self.estimatePose = np.matrix([self.bestParticle.x[0,0],self.bestParticle.y[0,0],self.bestParticle.yaw[0,0]])
        # calculate difference between last to positions
        deltaPose = trajectory[-1]-trajectory[-2]
        # calculate first estimate for new position
        estimatePose = trajectory[-1]+deltaPose

    def update(self, pointcloud, x, y, yaw):
        if first:
            first = False
            self.update_map(self, pointcloud, x, y, yaw)
            # add position to trajectory
            self.trajectory.append(self.startPose)
        else:
            pass

    def plot(self):
        # get position of all particles
        xy = [[p.x,p.y] for p in self.particles]
        x,y = zip(*xy)
        plt.imshow(gridslam.grid[:,:], interpolation ='none', cmap = 'binary')

##        # plot all particles
##        plt.scatter((y+self.offset[0])/self.resolution,(x+self.offset[0])/self.resolution,c='r',s=10,edgecolors='none',label='Partikel')

        # plot 10 best particles
##        plt.scatter((y[0:10]+self.offset[0])/self.resolution,(x[0:10]+self.offset[0])/self.resolution,c='y',s=20,edgecolors='none',label='Beste Partikel')
        plt.plot((self.estimatePose[0,0]+self.offset[0])/0.1, (self.estimatePose[0,1]+self.offset[0])/0.1,'.b')

##        plt.legend()
        plt.pause(0.0001)
##        plt.clf()
    

    def update_map(self, pointcloud, x, y, yaw):
        if pointcloud.any:

            # Позиция робота в системе координат ghj
            xPos = x
            yPos = y
            yaw = yaw
            #plt.axis('equal')
            # Матрица поворота 
            R = np.matrix([[np.cos(yaw),-np.sin(yaw)], [np.sin(yaw),np.cos(yaw)]])
            # Поворот массива координат лидара на угол робота
            pointcloud = pointcloud * np.transpose(R)

            pointcloud = pointcloud + np.matrix([xPos, yPos])     
               
            # offset of measurement in grid (x,y)
            # Смещение координат локальной системы в глобальную систему координат 
            offset = np.array([10.0,10.0])
            # translate pointcloud
            for ii in range(pointcloud.shape[0]):
                #если значение 0, значит препятствие не обнаружено, нужно отрисовать линию без препятсвия
                #print(pointcloud[ii,0])
                #print(pointcloud[ii,1])
                #d = math.sqrt((pointcloud[ii,0]-xPos)**2+(pointcloud[ii,1]-yPos)**2)
                #print(d)
                if abs(pointcloud[ii,0]-xPos) <= 0.05 or abs(pointcloud[ii,1]-yPos) <= 0.05:#если значение 0, значит препятствие не обнаружено
                    xi = math.ceil( (pointcloud[ii,0]+offset[0]) / self.resolution ) 
                    yi = math.ceil( (pointcloud[ii,1]+offset[1]) / self.resolution )
                    #print('zero')
                else:
                    # round points to cells
                    # Массив координат лидара в системе глобальных координат / размер ячейки = массив лидара
                    # в системе координат карты 
	                xi = math.ceil( (pointcloud[ii,0]+offset[0]) / self.resolution ) 
	                yi = math.ceil( (pointcloud[ii,1]+offset[1]) / self.resolution )
	                # set beam endpoint-cells as occupied 
	                if xi >= self.grid.shape[0] or yi >= self.grid.shape[1]:
	                	continue
	                self.grid[xi,yi] += self.l_occupied
	                # value > threshold? -> clamping 
	                if self.grid[xi,yi] > self.l_max:
	                    self.grid[xi,yi] = self.l_max

                x_start = math.ceil((xPos+offset[0])/self.resolution)
                y_start = math.ceil((yPos+offset[1])/self.resolution)
                
                # calculate cells between sensor and endpoint as free with bresenham
                startPos = np.array([[x_start,y_start]])
                endPos = np.array([[xi,yi]])
                bresenhamPath = bresenham.bresenham2D(startPos, endPos)
                
                # set free cells as free
                for jj in range(bresenhamPath.shape[0]):
                    path_x = int(bresenhamPath[jj,0])
                    path_y = int(bresenhamPath[jj,1])
                    
                    self.grid[path_x, path_y] += self.l_free
                    
                    # value < threshold? -> clamping
                    if self.grid[path_x, path_y] < self.l_min:
                        self.grid[path_x, path_y] = self.l_min


def scan2mapDistance(grid,pcl,offset,resolution):
    distance = 0;
    for i in range(pcl.shape[0]):
        # round points to cells
        xi = math.ceil( (pcl[i,0]+offset[0]) / resolution )
        yi = math.ceil( (pcl[i,1]+offset[0]) / resolution ) 
        distance += grid[xi,yi]
    return distance     

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
##    print(robot['scan'][0])
##    pointcloud = scan2pointcloud(robot['scan'][0])
##    plt.plot(pointcloud[:,0],pointcloud[:,1],'.b')
##    plt.pause(0.001)


##    for i in range(len(robot['scan'])):
##        pointcloud = scan2pointcloud(robot['scan'][i])
##        # Позиция робота в системе координат ghj
##        xPos = robot['x'][i]
##        yPos = robot['y'][i]
##        yaw = robot['yaw'][i]
##        #plt.axis('equal')
##        # Матрица поворота 
##        R = np.matrix([[np.cos(yaw),np.sin(yaw)], [-np.sin(yaw),np.cos(yaw)]])
##        # Поворот массива координат лидара на угол робота
##        pointcloud = pointcloud * np.transpose(R)
##        pointcloud = pointcloud + np.matrix([xPos, yPos])    
##        plt.plot(pointcloud[:,0],pointcloud[:,1],'.b')
##        plt.pause(0.001)
##        plt.clf()
    # load existing grid and filtered pointcloud
    # resolution of the grid


    trajectory = []

    # Позиция робота в системе координат ghj
    xPos = robot['x'][0]
    yPos = robot['y'][0]
    yaw  = robot['yaw'][0]
    
       
    pointcloud = scan2pointcloud(robot['scan'][0])
    # add measurement to grid
    gridslam = GridSLAM()
    gridslam.update_map(pointcloud, xPos, yPos, yaw)

    # add position to trajectory
    trajectory.append(gridslam.startPose)
##    plt.imshow(gridslam.grid[:,:], interpolation ='none', cmap = 'binary')
##    plt.pause(0.001)
    #создать начальную популяцию частиц
    gridslam.init_particles()
    # calculate difference between last to positions
    deltaPose = gridslam.trajectory[-1]-gridslam.trajectory[-2]
    # calculate first estimate for new position
    estimatePose = gridslam.trajectory[-1]+deltaPose
##    print('Difference of last two postitions:')
##    print('x='+str(deltaPose[0,0])+'m y='+str(deltaPose[0,1])+'m yaw='+str(deltaPose[0,2])+'rad')
##    print('First estimate for new position:')
##    print('x='+str(estimatePose[0,0])+'m y='+str(estimatePose[0,1])+'m yaw='+str(estimatePose[0,2])+'rad')
##    gridslam.plot()
    for i in range(len(robot['scan'])-1):
        pointcloud = scan2pointcloud(robot['scan'][i])
        gridslam.fitscan2Map(pointcloud)
        # calculate difference between last to positions
        deltaPose = gridslam.trajectory[-1]-gridslam.trajectory[-2]
        # calculate first estimate for new position
        estimatePose = gridslam.trajectory[-1]+deltaPose
        gridslam.update_map(pointcloud, estimatePose[0,0], estimatePose[0,1], estimatePose[0,2])
##        print('Difference of last two postitions:')
##        print('x='+str(deltaPose[0,0])+'m y='+str(deltaPose[0,1])+'m yaw='+str(deltaPose[0,2])+'rad')
##        print('First estimate for new position:')
##        print('x='+str(estimatePose[0,0])+'m y='+str(estimatePose[0,1])+'m yaw='+str(estimatePose[0,2])+'rad')
##        gridslam.plo3t()
    print('Done!')
    gridslam.plot()
##    # задать веса частицам
##    gridslam.calc_weight(pointcloud)
##    # выберем лучшие частицы для отсева
##    gridslam.Estimate()
##    # Resample
##    gridslam.Resample()
##    gridslam.calc_weight(pointcloud)
##    # add position to trajectory
##    gridslam.Estimate()
##    trajectory.append(gridslam.estimatePose)
##    # calculate difference of last to positions
##    deltaPose = trajectory[-1]-trajectory[-2]
##    # calculate first estimate for new position
##    estimatePose = trajectory[-1]+deltaPose
##    print('Difference of last two postitions:')
##    print('x='+str(deltaPose[0,0])+'m y='+str(deltaPose[0,1])+'m yaw='+str(deltaPose[0,2])+'rad')
##    print('First estimate for new position:')
##    print('x='+str(estimatePose[0,0])+'m y='+str(estimatePose[0,1])+'m yaw='+str(estimatePose[0,2])+'rad')
##
##    
    
##    # get position of all particles
##    xy = [[p.x,p.y] for p in particles]
##    x,y = zip(*xy)
##
##    # plot all particles
##    plt.scatter((y+offset[0])/resolution,(x+offset[0])/resolution,c='r',s=10,edgecolors='none',label='Partikel')
##
##    # plot 10 best particles
##    plt.scatter((y[0:10]+offset[0])/resolution,(x[0:10]+offset[0])/resolution,c='y',s=20,edgecolors='none',label='Beste Partikel')
##
##    # plot best estimate pointcloud
##    R = np.matrix([[np.cos(bestParticle.yaw),np.sin(bestParticle.yaw)],
##                   [-np.sin(bestParticle.yaw),np.cos(bestParticle.yaw)]])
##    pointcloudTransformed = pointcloud * np.transpose(R)
##    pointcloudTransformed = pointcloudTransformed + np.matrix([bestParticle.x,bestParticle.y])
##    plt.scatter([(pointcloudTransformed[:,1]+offset[0])/resolution],
##                [(pointcloudTransformed[:,0]+offset[0])/resolution],
##                 c='m',s=10,edgecolors='none',label='Punktwolke')
##    plt.legend()
##    plt.show()
##
##


##    #------------- RESAMPLE ----------------
##    # standard deviation of position and yaw for resampling
##    stddPosResample = stddPos/5.0
##    stddYawResample = stddYaw/5.0
##
##    # number of particels
##    nrParticleResample = 50
##    # delete old particles
##    particles.clear()
##
##    # create 50 particles for each of the best 10 particles
##    for bp in bestParticles:
##        for _ in range(0,nrParticleResample):
##            x = np.random.normal(bp.x,stddPosResample)
##            y = np.random.normal(bp.y,stddPosResample)
##            yaw = np.random.normal(bp.yaw,stddYawResample)
##
##            # create particle and append ist to list
##            p = Particle(x,y,yaw,1)
##            particles.append(p)
##    # weight all particles
##    for p in particles:
##        # rotate pointcloud
##        R = np.matrix([[np.cos(p.yaw),-np.sin(p.yaw)], [np.sin(p.yaw),np.cos(p.yaw)]])
##        pointcloudTransformed = pointcloud * np.transpose(R)
##        
##        # translate pointcloud
##        pointcloudTransformed = pointcloudTransformed + np.matrix([p.x,p.y])
##        
##        # weight particle
##        p.weight = scan2mapDistance(grid,pointcloudTransformed,offset,resolution)
##    
##    # sort particles
##    particles.sort(key = lambda Particle: Particle.weight,reverse=True)
####    
####    # plot grid
####
####    plt.imshow(grid[:,:], interpolation ='none', cmap = 'binary')
####
######    # get position of all particles
######    xy = [[p.x,p.y] for p in particles]
######    x,y = zip(*xy)
######
######    # plot all particles
######    plt.scatter((y+offset[0])/resolution,(x+offset[0])/resolution,c='r',s=10,edgecolors='none',label='Partikel')
######
######    # plot 10 best particles
######    plt.scatter((y[0:10]+offset[0])/resolution,(x[0:10]+offset[0])/resolution,c='y',s=20,edgecolors='none',label='Beste Partikel')
######
######    # plot best estimate pointcloud
######    R = np.matrix([[np.cos(bestParticle.yaw),np.sin(bestParticle.yaw)],
######                   [-np.sin(bestParticle.yaw),np.cos(bestParticle.yaw)]])
######    pointcloudTransformed = pointcloud * np.transpose(R)
######    pointcloudTransformed = pointcloudTransformed + np.matrix([bestParticle.x,bestParticle.y])
######    plt.scatter([(pointcloudTransformed[:,1]+offset[0])/resolution],
######                [(pointcloudTransformed[:,0]+offset[0])/resolution],
######                 c='m',s=10,edgecolors='none',label='Punktwolke')
######    plt.legend()
######    plt.show()
######    offsetStartPos = np.array([457797.930, 5428862.694])
####        
####            

    


        
