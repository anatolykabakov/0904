from protocol import openconnect, send_msg, recieve, process_data



class Serial(object):
    def __init__(self, port, speed):
        """Instantiate the object."""
        super(Serial, self).__init__()
        self.connect = self.open_connect(port, speed)
        
    def open_connect(self, port, speed):
        connect = openconnect(port, speed)
        return connect

    def closeconnect(self):
        self.connect.close()
        
    def getSerialData(self):
        data = recieve(self.connect)
        self.velocity, self.yaw, self.x, self.y = process_data(data)
        return self.velocity, self.yaw, self.x, self.y

    def setSerialData(self, velocity, omega):
        send_msg(self.connect, velocity, omega)
