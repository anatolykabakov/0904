from remote_api import Serial
from rplidar import RPLidar as Lidar
import math
import time
from utils import normalize_angle, scan2distVec
file = open("log.txt", "w")

class Robot:
    def __init__(self,wheel_radius , wheel_dist,  ARDUINO_HCR, LIDAR_DEVICE):
        # Connect to Arduino unit
        self.arduino   = Serial(ARDUINO_HCR, 57600)
        # Connect to Lidar unit
        self.lidar = Lidar(LIDAR_DEVICE)
        # Create an iterator to collect scan data from the RPLidar
        self.iterator = self.lidar.iter_scans()
        # First scan is crap, so ignore it
        next(self.iterator)
        self.lidar = None
        self.WHEELS_DIST = wheel_dist
        self.WHEELS_RAD  = wheel_radius
        self.x = 0
        self.y = 0
        self.yaw = 0
        self.linear_velocity = 0
        self.angular_velocity = 0
        self.omegaRight = 0
        self.omegaLeft = 0
        self.vr = 0
        self.vl = 0
        self.scan = []
        self.prev_time = time.time()
        self.current_time = time.time()
        self.dt = 0
        
        

    def update_state(self):
        self.current_time = time.time()
        self.dt = self.current_time - self.prev_time
        self.prev_time = self.current_time
        self.omegaRight = self.vr/self.WHEELS_RAD
        self.omegaLeft  = self.vl/self.WHEELS_RAD
        # фактическая линейная скорость центра робота
        self.linear_velocity = (self.WHEELS_RAD/2)*(self.omegaRight + self.omegaLeft);#//m/s
        # фактическая угловая скорость поворота робота
        self.angular_velocity = (self.WHEELS_RAD/self.WHEELS_DIST)*(self.omegaRight - self.omegaLeft);
        self.yaw+=(self.angular_velocity * self.dt)#;  #  // направление в рад
        self.yaw = normalize_angle(self.yaw)
        self.x += self.linear_velocity*math.cos(self.yaw) * self.dt# // в метрах
        self.y += self.linear_velocity*math.sin(self.yaw) * self.dt#

    def init_lidar(self):
        self.lidar = Lidar(LIDAR_DEVICE)
        # Create an iterator to collect scan data from the RPLidar
        self.iterator = lidar.iter_scans()
        # First scan is crap, so ignore it
        next(self.iterator)

    def stop(self):
        # Shut down the lidar connection
        print('Stoping.')
        file.close()
        self.arduino.setSerialData(0,0)
        self.arduino.close_connect()
        self.lidar.stop()
        self.lidar.disconnect()

    def vRToDrive(self, vLinear, vAngular):
        return ((2 * vLinear) + (self.WHEELS_DIST * vAngular)) / 2

    def vLToDrive(self, vLinear, vAngular):
        return ((2 * vLinear) - (self.WHEELS_DIST * vAngular))/ 2

    def drive(self, vLinear, vAngular):
        vr = self.vRToDrive(vLinear, vAngular)
        vl = self.vLToDrive(vLinear, vAngular)
        self.arduino.setSerialData(round(vr,1), round(vl,1))

    def sense(self):
        #get odometry data
        self.vr, self.vl = self.arduino.getSerialData()
        
        #get laser dara
        # Extract (quality, angle, distance) triples from current scan
        self.scan = [[item[1], item[2]] for item in next(self.iterator)]

    def write_log(self):
        dist_vec = scan2distVec(self.scan)
        log_data = str(round(self.current_time, 2))+' '+ str(round(self.vr, 2))+ ' ' + str(round(self.vl, 2))+ ' ' + str(round(self.x, 2))+' '+str(round(self.y, 2))+' '+str(round(self.yaw, 2))+' '+' '.join(str(el) for el in dist_vec)+'\n'
        file.write(log_data)
        
