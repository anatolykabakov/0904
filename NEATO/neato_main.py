import time
import math
from robot import Robot

NEATO_PORT             = '/dev/ttyACM1'
# These are sensible values for an ad-hoc wifi network
PORT_SPEED = 115200
# Serial-port timeout
TIMEOUT_SEC  = 0.1

if __name__ == '__main__':
    robot = Robot(NEATO_PORT, PORT_SPEED, TIMEOUT_SEC)

    while True:
        robot.sense()
        robot.update_state()
        robot.drive(0.2, 0.5)
