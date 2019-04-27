import serial
import time
from rplidar import RPLidar

#Arduino serial port
ARDUINO_PORT = '/dev/ttyACM0'
ARDUINO_SPEED = 115200

#RPLidar serial port
LIDAR_PORT = '/dev/ttyUSB0'

#Arduino protocol
set_command = 'v'
print_command = 'd'
start_connect = 's'

class hcr():
    def __init__(self, ARDUINO_PORT='/dev/ttyACM0', LIDAR_PORT='/dev/ttyUSB0'):
        self.connect = self.openconnect(ARDUINO_PORT, ARDUINO_SPEED)
        # Connect to Lidar unit
        self.lidar = RPLidar(LIDAR_PORT)
        # Create an iterator to collect scan data from the RPLidar
        self.iterator = self.lidar.iter_scans()

    def check_connect(self, connect):
        c = connect.read(1).decode()
        if c != 'c':
            print("false read")

    def openconnect(self, port, speed):
        connect = serial.Serial(port, speed)
        time.sleep(1)
        while not connect.is_open:
            self.openconnect(port, speed)
        is_connected = False
        while not is_connected:
            print("Waiting for arduino...")
            connect.write(start_connect.encode())
            connect_flag = connect.read(1).decode()
            self.check_connect(connect)
            if not connect_flag:
                time.sleep(0.1)
                continue
            if connect_flag == 'r':
                is_connected = True
                print('Connected!')
        return connect
        
    def send(self, lvel, avel):
        send_data = set_command + str(round(lvel,2)) + ' ' + str(round(avel,2)) + "\n"
        self.connect.write(send_data.encode())
        self.check_connect(self.connect)

    def lidar_stop(self):
        self.lidar.stop()
        self.lidar.stop_motor()
        self.lidar.disconnect()

    def arduino_stop(self):
        self.setMotors(0,0)
        self.connect.close()

    def stop(self):
        self.lidar_stop()
        self.arduino_stop()

    def getMotors(self):
        self.connect.write(print_command.encode())
        data = self.connect.read(12).decode()
        #print(data) 
        self.check_connect(self.connect)
        data = data.split(';')
        right = float(data[0])
        left = float(data[1])
        return right, left
 
    def setMotors(self, rightVelocity, leftVelocity):
        self.send(rightVelocity, leftVelocity)
        #print('2')
        #self.check_connect(self.connect)

    def getScan(self):
        #get laser dara
        # Extract (quality, angle, distance) triples from current scan
        scan = [[item[1], item[2]] for item in next(self.iterator)]
        return scan
    


if __name__ == '__main__':

    robot = hcr(ARDUINO_PORT, LIDAR_PORT)
    while True:
        try:
            right, left = robot.getMotors()
            print(right,left)
            scan = robot.getScan()
            print(scan)
            robot.setMotors(0.2,0.2)
            print('1')
        except KeyboardInterrupt:
            robot.stop()
