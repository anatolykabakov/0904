
from lib import Robot, Serial, Planning, Tracking

ax = [0.0, 1, 1, 2, 2]
ay = [0.0, 0, 1, 1, 0]
dt = 0
if __name__ == '__main__':
    #----Init state--

    serial   = Serial('com4', 57600)
    robot    = Robot(x=0.0, y=0.0, yaw=0.0, v=0.0, omega=0, dt=dt)
    planning = Planning()
    tracking = Tracking(robot, planning, serial=serial, mode='robot', dt = dt)
    #----Planning----
    cx, cy, cyaw = planning.getTrack(ax, ay)
    #----Tracking----
    x, y         = tracking.motioncontrol(cx, cy, cyaw, target_speed=0.1)










 


