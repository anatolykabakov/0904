from protocol import openconnect, closeconnect, send_msg, recieve, process_data



class Serial(object):
    def __init__(self, port, speed):
        """Instantiate the object."""
        super(Serial, self).__init__()
        self.connect = self.open_connect(port, speed)
        
    def open_connect(self, port, speed):
        connect = openconnect(port, speed)
        return connect

    def close_connect(self):
        closeconnect(self.connect)
        
    def getSerialData(self):
        data = recieve(self.connect)
        return process_data(data)
 
    def setSerialData(self, rightVelocity, leftVelocity):
        send_msg(self.connect, rightVelocity, leftVelocity)
