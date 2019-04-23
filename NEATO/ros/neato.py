
from math import sin,cos


from neato_driver import xv11, BASE_WIDTH, MAX_SPEED
port = '/dev/ttyACM0'


if __name__ == "__main__":    
    robot = xv11(port)
    while True:
        try:
            #self.robot.requestScan()
            ranges = robot.getScanRanges()

            # get motor encoder values
            left, right = robot.getMotors()
            
            ldist, rdist, linear_vel = 100, 100, 100
            # send updated movement commands
            robot.setMotors(ldist, rdist, linear_vel)
            
            # ask for the next scan while we finish processing stuff
            robot.requestScan()

            time.sleep(0.1)
        except KeyboardInterrupt:
            # shut down
            robot.setLDS("off")
            robot.setTestMode("off") 

