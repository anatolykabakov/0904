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
        self.arduino = self.openconnect(ARDUINO_PORT, ARDUINO_SPEED)
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
        send_data = set_command + str(lvel) + ' ' + str(avel) + "\n"
        self.arduino.write(send_data.encode())
        self.check_connect(self.arduino)

    def lidar_stop(self):
        self.lidar.stop()
        self.lidar.disconnect()

    def arduino_stop(self):
        self.setMotors(0,0)
        self.arduino.close()

    def stop(self):
        self.arduino_stop()
        self.lidar_stop()

    def getMotors(self):
        self.arduino.write(print_command.encode())
        data = self.arduino.read(12).decode() # чтение строки из 24 символов в строку
        self.check_connect(self.arduino)
        data = data.split(';')
        right = float(data[0])
        left = float(data[1])
        return right, left
 
    def setMotors(self, rightVelocity, leftVelocity):
        self.send(rightVelocity, leftVelocity)
        self.check_connect(self.arduino)

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
            right, left = robot.getMotors()
            scan = robot.getScan()
            robot.setMotors(0.2,0.2)
            print(encoders)
            print(scan)
        except KeyboardInterrupt:
            robot.stop()
