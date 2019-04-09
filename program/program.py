from remote_api import Serial
from rplidar import RPLidar as Lidar
LIDAR_DEVICE            = '/dev/ttyUSB0'
ARDUINO_HCR             = '/dev/ttyACM0'

if __name__ == '__main__':
    # Connect to Arduino unit
    arduino   = Serial(ARDUINO_HCR, 57600)
    # Connect to Lidar unit
    lidar = Lidar(LIDAR_DEVICE)
    
    # Create an iterator to collect scan data from the RPLidar
    iterator = lidar.iter_scans()

    # We will use these to store previous scan in case current scan is inadequate
    previous_distances = None
    previous_angles    = None

    # First scan is crap, so ignore it
    next(iterator)
    for i in range(20):
        # Extract (quality, angle, distance) triples from current scan
        items = [item for item in next(iterator)]
        for item in items:
            print(item)

        ## Extract distances and angles from triples
        #distances = [item[2] for item in items]
        #angles    = [item[1] for item in items]
        #print()
    # Shut down the lidar connection
    lidar.stop()
    lidar.disconnect()
