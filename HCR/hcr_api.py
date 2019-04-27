import serial
import time
from rplidar import RPLidar

#Arduino serial port
ARDUINO_PORT = '/dev/ttyACM0'
ARDUINO_SPEED = 57600

#RPLidar serial port
LIDAR_PORT = '/dev/ttyUSB0'

#Arduino protocol
set_command = 'v'
print_command = 'd'
start_connect = 's'

class hcr():
    def __init__(self, ARDUINO_PORT='/dev/ttyACM0', LIDAR_PORT='/dev/ttyUSB0'):
        self.arduino = serial.Serial(ARDUINO_PORT, ARDUINO_SPEED)
        # Connect to Lidar unit
        self.lidar = RPLidar(LIDAR_DEVICE)
        # Create an iterator to collect scan data from the RPLidar
        self.iterator = self.lidar.iter_scans()
        
    def send(self, lvel, avel):
        send_data = set_command + str(lvel) + ' ' + str(avel) + "\n"
        self.arduino.write(send_data.encode())

    def lidar_stop(self):
        self.lidar.stop()
        self.lidar.disconnect()
        self.arduino.close()

    def stop(self):
        self.setMotors(0,0)
        self.lidar_stop()

    def getMotors(self):
        data = self.recieve()
        data = data.split(';')
        right = float(data[0])
        left = float(data[1])
        return right, left
 
    def setMotors(self, rightVelocity, leftVelocity):
        self.send(rightVelocity, leftVelocity)

    def getScan(self):
        #get laser dara
        # Extract (quality, angle, distance) triples from current scan
        scan = [[item[1], item[2]] for item in next(self.iterator)]
        return scan
    
    def recieve(self):
        line = b''
        while True:
            data = self.arduino.read()
            line+=data
            if data==b'c':
                break
        return line

if __name__ == '__main__':

    robot = hcr(ARDUINO_PORT, LIDAR_PORT)
    while True:
        try:
            encoders = robot.getMotors()
            scan = robot.getScan()
            robot.setMotors(0.2,0.2)
            print(encoders)
            print(scan)
        except KeyboardInterrupt:
            robot.stop()
