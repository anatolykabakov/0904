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
        return data
    def getScan(self):
        self.connect.write('GetLDSScan\n'.encode())
        scan = self.read().decode().split('\r\n')
        return scan
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
    
