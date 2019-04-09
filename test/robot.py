
import math
import time
from utils import normalize_angle

class Robot_Diff(object):
    def __init__(self, x, y, yaw, v, radius, length, dt):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.omega = 0
        self.radius = radius
        self.length = length

        self.prev_time = 0
        self.current_time = 0
        self.delta_time = dt
        
    def update(self, v, omega):
##        self.d_time()
        
##        omegaRight = speed_right/self.radius#   mm/s / mm =  rad за сек
##        omegaLeft  = speed_left /self.radius#
##
##        # фактическая линейная скорость центра робота
##        self.v     = (self.radius/2)*(omegaRight + omegaLeft)#/1000#;//mm/s * 1000 -> M/S
##        self.v += acceleration * dt
##        # фактическая угловая скорость поворота робота
##        self.omega = (self.radius/self.length)*(omegaRight - omegaLeft)
        self.v = v
        self.omega = omega
        self.yaw += self.delta_time * self.omega
        self.yaw  = normalize_angle(self.yaw)
        self.x   += self.delta_time * self.v * math.cos(self.yaw)
        self.y   += self.delta_time * self.v * math.sin(self.yaw)
        
        
    def getPos(self):
        return self.x, self.y, self.yaw
    
    def d_time(self):
        self.prev_time = self.current_time
        self.current_time = time.time()
        self.delta_time = self.current_time - self.prev_time
        print(self.delta_time)
