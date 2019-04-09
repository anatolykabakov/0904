from remote_api import Serial
from rplidar import RPLidar as Lidar
import time
LIDAR_DEVICE            = '/dev/ttyUSB0'
ARDUINO_HCR             = '/dev/ttyACM0'
file = open("log.txt", "w")

def scan2distVec(scan):
    scanSize = 360
    distVec = [0 for i in range(scanSize)]

    for point in scan: # create breezySLAM-compatible data from raw scan data
      dist = point[1]
      index = int(point[0])
      if not 0 <= index < scanSize: continue
      distVec[index] = int(dist)
    return distVec

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
    #timer
    prev_time = 0
    current_time = time.time()
    full_time = 0
    cur_time = 0
    while full_time <= 60:

        prev_time = current_time
        current_time = time.time()
        delta_time = current_time - prev_time
        full_time += delta_time
        
        # Extract (quality, angle, distance) triples from current scan
        items = [[item[1], item[2]] for item in next(iterator)]
        v, yaw, x, y = arduino.getSerialData()

        distVec = scan2distVec(items)
        log_arduino = str(round(v, 2))+' '+str(round(yaw, 2))+' '+str(round(x, 2))+' '+str(round(y, 2))
        lidar_log = ' '.join(str(el) for el in distVec)
        file.write(str(current_time)+' '+log_arduino+' '+lidar_log+'\n')
        arduino.setSerialData(0.3, 0.5)
        current_time
            
    # Shut down the lidar connection
    print('Stoping.')
    file.close()
    lidar.stop()
    lidar.disconnect()
    arduino.setSerialData(0,0)
