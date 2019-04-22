import math
import time
from utils import normalize_angle
from remote_api_neato import Neato_API
# Robot  params
ROBOT_HEIGHT_MM = 500
ROBOT_WIDTH_MM  = 300
BASE_WIDTH = 248    # millimeters
WHEEL_RADIUS = 47
MAX_SPEED = 300     # millimeters/second

class Robot(object):
    def __init__(self, port, speed, timeout):
        self.x = 0
        self.y = 0
        self.yaw = 0
        self.v = 0
        self.vl = 0
        self.vr = 0
        self.radius = WHEEL_RADIUS/1000
        self.length = ROBOT_WIDTH_MM/1000

        self.neato_api = Neato_API(port, speed, timeout)

        self.prev_time = 0
        self.current_time = 0
        self.delta_time = 0
        
    def update_state(self):
        self.d_time()
        omegaRight = self.vr/self.radius#  rad за сек
        omegaLeft  = self.vl /self.radius#

        # фактическая линейная скорость центра робота
        V     = (self.radius/2)*(omegaRight + omegaLeft)#/1000#;//mm/s * 1000 -> M/S
        # фактическая угловая скорость поворота робота
        omega = (self.radius/self.length)*(omegaRight - omegaLeft)
        self.yaw += self.delta_time * omega
        self.yaw  = normalize_angle(self.yaw)
        self.x   += self.delta_time * V * math.cos(self.yaw)
        self.y   += self.delta_time * V * math.sin(self.yaw)
        
    def sense(self):
        #get odometry data
        wheel = self.neato_api.getMotor()
        self.vr = wheel['RightWheel_Speed']/1000
        self.vl = wheel['LeftWheel_Speed']/1000)

        #get laser data
        scan = self.neato_api.getScan()
    
    def d_time(self):
        self.prev_time = self.current_time
        self.current_time = time.time()
        self.delta_time = self.current_time - self.prev_time
        #print(self.delta_time)

    def vRToDrive(self, vLinear, vAngular):
        return ((2 * vLinear) + (self.WHEELS_DIST * vAngular)) / 2

    def vLToDrive(self, vLinear, vAngular):
        return ((2 * vLinear) - (self.WHEELS_DIST * vAngular))/ 2

    def drive(self, vLinear, vAngular):
        vr = self.vRToDrive(vLinear, vAngular)
        vl = self.vLToDrive(vLinear, vAngular)
        leftDist  = round(vl/self.delta_time,2)*1000
        rightDist = round(vr/self.delta_time,2)*1000
        speed = round(vLinear,2)*1000
        if speed >= 300:
            speed = 300
        self.neato_api.setMotors(self, leftDist, rightDist, speed)
