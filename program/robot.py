import time
import math
from utils import normalize_angle
class Robot:
    def __init__(self, wheel_dist, wheel_radius):
        self.WHEELS_DIST = wheel_dist
        self.WHEELS_RAD  = wheel_radius
        self.x = 0
        self.y = 0
        self.yaw = 0
        self.linear_velocity = 0
        self.angular_velocity = 0
        self.omegaRight = 0
        self.omegaLeft = 0
        self.prev_time = time.time()
        self.current_time = time.time()
        self.dt = 0

    def update_state(self, vr, vl):
        self.current_time = time.time()
        self.dt = self.current_time - self.prev_time
        self.prev_time = self.current_time
        self.omegaRight = vr/self.WHEELS_RAD
        self.omegaLeft  = vl/self.WHEELS_RAD
        # фактическая линейная скорость центра робота
        self.linear_velocity = (self.WHEELS_RAD/2)*(self.omegaRight + self.omegaLeft);#//m/s
        # фактическая угловая скорость поворота робота
        self.angular_velocity = (self.WHEELS_RAD/self.WHEELS_DIST)*(self.omegaRight - self.omegaLeft);
        self.yaw+=(self.angular_velocity * self.dt)#;  #  // направление в рад
        self.yaw = normalize_angle(self.yaw)
        self.x += self.linear_velocity*math.cos(self.yaw) * self.dt# // в метрах
        self.y += self.linear_velocity*math.sin(self.yaw) * self.dt#

    def stop(self):
        self.lWheel.stop()
        self.rWheel.stop()

    def vRToDrive(self, vLinear, vAngular):
        return (((2 * vLinear) + (self.WHEELS_DIST * vAngular)) / (2));

    def vLToDrive(self, vLinear, vAngular):
        return (((2 * vLinear) - (self.WHEELS_DIST * vAngular)) / (2));

    def drive(self, vLinear, vAngular):
        self.lWheel.setSpeed(self.__vLToDrive(vLinear, vAngular))
        self.rWheel.setSpeed(self.__vRToDrive(vLinear, vAngular))
