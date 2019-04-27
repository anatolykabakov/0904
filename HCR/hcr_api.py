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
        self.connect = self.openconnect(ARDUINO_PORT, ARDUINO_SPEED)
        # Connect to Lidar unit
        self.lidar = RPLidar(LIDAR_DEVICE)
        # Create an iterator to collect scan data from the RPLidar
        self.iterator = self.lidar.iter_scans()

    def check_connect(self):
        c = self.connect.read(1).decode()
        if c != 'c':
            print("false read")
        
    def send(self, lvel, avel):
        send_data = set_command + str(lvel) + ' ' + str(avel) + "\n"
        self.connect.write(send_data.encode())
        self.check_connect(connect)

    def recieve(self):
        self.connect.write(print_command.encode())
        recieve_data = self.connect.read(12).decode() # чтение строки из 24 символов в строку
        self.check_connect()
        return recieve_data

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

    def closeconnect(self):
        self.connect.close()

    def lidar_stop(self):
        self.lidar.stop()
        self.lidar.disconnect()

    def getMotors(self):
        data = self.recieve()
        data = data.split(';')
        right = float(data[0])
        left = float(data[1])
        return right, left
 
    def setSerialData(self, rightVelocity, leftVelocity):
        self.send(rightVelocity, leftVelocity)

    def getLidarScan(self):
        #get laser dara
        # Extract (quality, angle, distance) triples from current scan
        scan = [[item[1], item[2]] for item in next(self.iterator)]
        return scan

if __name__ == '__main__':
    #Arduino serial port
    ARDUINO_PORT = '/dev/ttyACM0'

    #RPLidar serial port
    LIDAR_PORT = '/dev/ttyUSB0'
    robot = hcr(ARDUINO_PORT, LIDAR_PORT)
    while True:
        try:
            encoders = robot.getMotors()
            scan =    robot.getScan()
            robot.setMotors(100,100,100)
            print(encoders)
            print(scan)
        except KeyboardInterrupt:
            robot.stop()
