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
        
    def docommand(self, port, command):
        to_port = command + '\n'
        port.write(to_port.encode())

    def connect(self, port, speed, timeout):
        return serial.Serial(port, speed, serial.EIGHTBITS, serial.PARITY_NONE, serial.STOPBITS_ONE, timeout)
            
    def enableTestMode(self):
        # Put the XV-11 into test mode
        self.docommand(self.robot, 'TestMode on')
        
    def LidarOn(self):
        # Spin up the LIDAR
        self.docommand(self.robot, 'SetLDSRotation on')

    def LidarOff(self):
        # Spin up the LIDAR
        self.docommand(self.robot, 'SetLDSRotation off')
        
    # Get LIDAR scan. 
    # Returns scan as a list of (angle, distance, intensity) tuples
    def getScan(self):
        scandata = None
            
        # Robot sends scaled LSD sensor values scaled to (0,1)
        if self.robot:
        
            # Run scan command
            self.docommand(self.robot, 'GetLDSScan')
                    
            # Grab scan results till CTRL-Z
            scandata = self.robot.read(MAX_SCANDATA_BYTES)
            scandata = scandata.decode()
                        
        # Stubbed version sends constant distances
        else:
            scandata = ''
            for k in range(360):
                scandata = scandata + str(k) + ',1500,100,0\n'  

                                
        # Parse the scan into a list of tuples
        scanvals = []
        for line in scandata.split('\n'):
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
    def getMotor(self):
        wheel = {}

        command = 'GetMotors' 
        if self.robot:
            self.docommand(self.robot, command) 
            line = self.robot.read(647)
            
        line      = line.decode().split('\r\n')
        
        for l in line[2:-1]:
            r = l.split(',')
            name = r[0]
            value = r[1]
            wheel[name] = int(value)

        return wheel

        
    # Send robot server a mesage to set motor distances and speed
    def setMotors(self, leftDist, rightDist, speed):
        message = 'm ' + str(leftDist) + ' ' + str(rightDist) + ' ' + str(speed)
        command = 'SetMotor' + message[1:]    
                                    
        if self.robot:
            self.docommand(self.robot, command)
        
        else:
            print(command)
