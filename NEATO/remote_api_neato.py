import serial
#import socket
import time
import sys



# These are sensible values for an ad-hoc wifi network
PORT = '/dev/ttyACM0'               # Raspbian
PORT_SPEED = 115200
# Serial-port timeout
TIMEOUT_SEC  = 0.1

# These are arbitrary
MAX_SCANDATA_BYTES = 10000
class Neato_API(object):
    # Constructor opens socket to server
    def __init__(self, port, speed, timeout):
        self.robot = self.connect(port, speed, timeout)
        self.enableTestMode()
        self.LidarOn()
        
    def send(self, command):
        self.robot.write(command.encode())

    def connect(self, port, speed, timeout):
        return serial.Serial(port, speed, serial.EIGHTBITS, serial.PARITY_NONE, serial.STOPBITS_ONE, timeout)
            
    def enableTestMode(self):
        # Put the XV-11 into test mode
<<<<<<< HEAD
        self.send('TestMode on\n')
        
    def LidarOn(self):
        # Spin up the LIDAR
        self.send('SetLDSRotation on\n')

    def LidarOff(self):
        # Spin up the LIDAR
        self.send('SetLDSRotation off\n')
=======
        self.send(self.robot, 'TestMode on')
        
    def LidarOn(self):
        # Spin up the LIDAR
        self.send(self.robot, 'SetLDSRotation on')

    def LidarOff(self):
        # Spin up the LIDAR
        self.send(self.robot, 'SetLDSRotation off')
>>>>>>> 813af25a351a8155968870f98bace952f7d2899e

    def recieve_data(self):
        stop_bit = b'\x1a'
        line = b''
        first = False
        while True:
            data = self.robot.read(1)
            if data == stop_bit:
                break
            line += data

            #print(line)
        #line = self.robot.read(300)
        return line
        
    # Get LIDAR scan. 
    # Returns scan as a list of (angle, distance, intensity) tuples
    def getScan(self):
        scandata = None
            
        # Run scan command
        self.send('GetLDSScan\n')
                
        # Grab scan results till CTRL-Z
        scandata = self.recieve_data()
        scandata = scandata.decode().split('\r\n')
                                              
        # Parse the scan into a list of tuples
        scanvals = []
        for line in scandata[2:-2]:
            try:
                vals = line.split(',')
                # Only use legitimate scan tuples with zero error
                if len(vals) == 4 and not int(vals[3]):
                    angle = int(vals[0])
                    distance = int(vals[1])
                    intensity = int(vals[2])
                    scanvals.append([angle, distance])
                    
            except:
                None
                
        return scanvals
    def getEncoders(self):
        wheel = {}

        command = 'getmotors\n' 
        
<<<<<<< HEAD
        self.send(command)
        #time.sleep(0.01)
        #self.robot.flushInput() 
=======
        self.send(command) 
>>>>>>> 813af25a351a8155968870f98bace952f7d2899e
        line = self.recieve_data().decode().split('\r\n')
        print(line)
        line = self.recieve_data().decode().split('\r\n')
        print(line)
        for l in line:
            r = l.split(',')
            name = r[0]
            value = r[1]
            wheel[name] = int(value)

        rightEncoders = wheel['RightWheel_Speed']/1000
        leftEncoders  = wheel['LeftWheel_Speed']/1000

        return rightEncoders, leftEncoders

        
    # Send robot server a mesage to set motor distances and speed
    def setMotors(self, leftDist, rightDist, speed):
        message = str(leftDist) + ' ' + str(rightDist) + ' ' + str(speed)
        command = 'SetMotor' + message[1:]    
                                    
        if self.robot:
            self.docommand(self.robot, command)
        
        else:
            print(command)
