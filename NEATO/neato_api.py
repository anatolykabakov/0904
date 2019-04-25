import serial
import time

class xv21(object):
    def __init__(self,port):
        self.connect = serial.Serial(port, 115200)
        self.connect.write('TestMode on\n'.encode())
        clear_buff = self.read()
     
    def getMotors(self):
        self.connect.write('GetMotors\n'.encode())
        
        data = self.read()
        data = data.decode().split('\r\n')
        for l in line[2:-1]:
            r = l.split(',')
            name = r[0]
            value = r[1]
            wheel[name] = int(value)

        right = wheel['RightWheel_Speed']/1000
        left  = wheel['LeftWheel_Speed']/1000
        
        return right, left
    
    def getScan(self):
        scanvals=[]
        self.connect.write('GetLDSScan\n'.encode())
        scan = self.read().decode().split('\r\n')
        for line in scan[2:-2]:
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
    
    def setMotors(self, l, r, s):
        """ Set motors, distance left & right + speed """
        
        if (int(l) == 0 and int(r) == 0 and int(s) == 0):
            if (not self.stop_state):
                self.stop_state = True
                l = 1
                r = 1
                s = 1
        else:
            self.stop_state = False
        command = "setmotor "+str(int(l))+" "+str(int(r))+" "+str(int(s))+"\n"
        self.port.write(command.encode())
        
    def read(self):
        line = b''
        while True:
            data = self.connect.read()
            line+=data
            if data==b'\x1a':
                break
        return line




        
if __name__ == '__main__':
    #neato = serial.Serial('/dev/ttyACM1',115200,timeout=0)
    #neato.write('TestMode on\n'.encode())
    #neato.write('GetMotors\n'.encode())
    #time.sleep(1)
    #data = neato.read(neato.inWaiting())
    #time_last = time.time()
    #data = read(neato)
    #data = read(neato)
    #now = time.time() - time_last
    #neato.write('GetLDSScan\n'.encode())
    #data=read(neato)
    #data=read(neato)
    #print(now)
    robot = xv21('/dev/ttyACM1')
    encoders = robot.getMotors()
    scan =    robot.getScan()
    print(encoders)
    print(scan)
    
