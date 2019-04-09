import time
from remote_api import Serial
file = open('log_vel_nopid.txt', 'w')

if __name__ == '__main__':
    serial   = Serial('com3', 57600)
    prev_time = 0
    current_time = time.time()
    full_time = 0
    cur_time = 0
    while full_time <= 5:
        v, yaw, x, y = serial.getSerialData()
        prev_time = current_time
        current_time = time.time()
        delta_time = current_time - prev_time
        #while delta_time*1000 <= 10:
        #    current_time = time.time()
        #    delta_time = current_time - prev_time
        print(delta_time*1000)
        full_time += delta_time
        #robot.yaw = normalize_angle(robot.yaw)
        serial.setSerialData(0.3, 0)
        log_data = str(round(full_time, 2))+' '+str(round(v, 2))+'\n'
        file.write(log_data)
    serial.setSerialData(0, 0)
    file.close()
    print('Done!')
