#from rplidar import RPLidar
##lidar = RPLidar('/dev/ttyUSB0')
iterator = lidar.iter_scans()



class Sensors(object):
    def __init__(self, lidar_port):
	    self.lidar_port = lidar_port
	    self.iterator = None
	    self.lidar = None
	    pass

	def start_lidar(self):
		self.lidar = RPLidar(self.lidar_port)
		self.iterator = self.lidar.iter_scans()

	def stop_lidar(self):
		self.lidar.stop()
		self.stop_motor()
		self.lidar.disconnect()

	def get_scan(self):
		scan = [[angle, dist] for quality, angle, dist in next(self.iterator)]
		return scan