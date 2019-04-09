import matplotlib.pyplot as plt


if __name__ == '__main__':
    file = open("log_vel_nopid.txt", "r")
    log = []
    time_log = []
    value_log = []
    for line in file:
        line = line.split(' ')
        time = float(line[0])
        value = float(line[1])
        log.append([time, value])
        time_log.append(time)
        value_log.append(value)
    
    plt.plot(time_log, value_log, "-b")
    plt.xlabel('время, сек')
    plt.ylabel('скорость метки/интервал времени')
    plt.show()
        
