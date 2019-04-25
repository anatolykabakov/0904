import math
from neato_api import xv21
BASE_WIDTH = 248    # millimeters
MAX_SPEED = 300     # millimeters/second0000

class Neato(object):
    def __init__(self, port):
        self.api = xv21(port)
        self.last_encoders = []
        self.encoders_current = []
        self.scan
        self.x = 0
        self.y = 0
        self.th = 0
        self.prev_time = time.time()
        self.current_time = time.time()
        self.dt = 0
        self.lvel = 0
        self.avel = 0

    def sense(self):
        self.encoders_current = self.api.getMotors()
        self.scan     = self.api.getScan()

    def update_state(self):
        self.current_time = time.time()
        self.dt = self.current_time - self.prev_time
        self.prev_time = self.current_time
        left=self.encoders_current[1]
        right=self.encoders_current[0]
        d_left = (left - self.last_encoders[0])/1000.0
        d_right = (right - self.last_encoders[1])/1000.0
        self.last_encoders = [left, right]
        
        dx = (d_left+d_right)/2
        dth = (d_right-d_left)/(BASE_WIDTH/1000.0)

        x = math.cos(dth)*dx
        y = -math.sin(dth)*dx
        self.x += math.cos(self.th)*x - math.sin(self.th)*y
        self.y += math.sin(self.th)*x + math.cos(self.th)*y
        self.th += dth
        self.lvel = dx/self.dt
        self.avel = dth/self.dt

    def drive(self, lvel, avel):
        vl = (2*lvel - avel*(BASE_WIDTH/1000))/2
        vr = (2*lvel - avel*(BASE_WIDTH/1000))/2
        self.api.setMotors(100,100,100)

    def stop(self):
        self.api.stop()
        

if __nama__ == '__main__':
    robot = Neato('/dev/ttyACM1')
    while True:
        try:
            robot.sense()
            robot.update_state()
            robot.drive(0.2, 1)
        except KeyboardInterrupt:
            robot.stop()
