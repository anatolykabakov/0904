from remote_api import Serial
from rplidar import RPLidar as Lidar
import time
import math
LIDAR_DEVICE            = '/dev/ttyUSB0'
ARDUINO_HCR             = '/dev/ttyACM0'
file = open("log.txt", "w")


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
    Ka = 1.5 # коэффициент притяжения
    Kr = 1.0 # коэффициент отталкивания
    dg = math.sqrt((xg-xr)**2+(yg-yr)**2)#расстояние до цели
    if dg <= 0.1:
        stop = True
    else:
        stop = False
    Xrf = []
    Yrf = []

    #Цикл по всем измерениям лидара
    for point in scan:
        alpha = point[0] #угол измерения лидара градусы
        dist  = point[1] #дистанция измерения лидара миллиметры
        alpha = alpha*math.pi/180 #градусы в радианты
        dist  = dist/1000 # миллиметры в метры
        
        if 0<alpha<math.pi/2:
            gamma = alpha
        if 3*math.pi/4 < alpha < 2*math.pi:
            gamma = alpha-2*math.pi
        # Множество отталкивания
        if dist <= rd:
            f = ((rd - dist)/dist)**2
        if dist > rd:
            f = 0
        #Вектор отталкивания единичного препятс
        Xrf.append(f*math.cos(gamma))
        Yrf.append(f*math.sin(gamma))
        
    #Общий вектор отталкивания
    RF = [sum(Xrf), sum(Yrf)]
    #Вектор притяжения
    Xra = (xg-xr)/dg
    Yra = (yg-yr)/dg
    RA = [Xra,Yra]
    #Вектор движения
    Xrf = RF[0]
    Yrf = RF[1]
    RV_x = Xra*Ka-Xrf*Kr
    RV_y = Yra*Ka-Yrf*Kr

    Vu = 0.3#Желаемая скорость движения

    LinearVelocity = Vu*math.sqrt(RV_x**2+RV_y**2)
    AngularVelocity = math.atan2(RV_y, RV_x)
    
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
    # Connect to Arduino unit
    arduino   = Serial(ARDUINO_HCR, 57600)
    # Connect to Lidar unit
    lidar = Lidar(LIDAR_DEVICE)
    
    # Create an iterator to collect scan data from the RPLidar
    iterator = lidar.iter_scans()

    Vu = 0.3
    xg, yg = 1, 2


    # First scan is crap, so ignore it
    next(iterator)
    #timer
    prev_time = 0
    current_time = time.time()
    full_time = 0
    cur_time = 0
    stop = False
    while !stop:
        v, yaw, x, y = arduino.getSerialData()
        # Extract (quality, angle, distance) triples from current scan
        items = [[item[1], item[2]] for item in next(iterator)]
        LinearVelocity, AngularVelocity,stop = potencial_field(x, y, xg, yg, Vu, items)
        arduino.setSerialData(LinearVelocity, AngularVelocity)
            
    # Shut down the lidar connection
    print('Stoping.')
    file.close()
    lidar.stop()
    lidar.disconnect()
    arduino.setSerialData(0,0)
