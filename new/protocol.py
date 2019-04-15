
import serial
import time

set_command = 'v'
print_command = 'd'
start_connect = 's'

def check_connect(connect):
    c = connect.read(1).decode()
    if c != 'c':
        print("false read")
        
def send_msg(connect, a, b):
    send_data = set_command + str(a) + ' ' + str(b) + "\n"
##    print(send_data)
    connect.write(send_data.encode())
    check_connect(connect)

def recieve(connect):
    connect.write(print_command.encode())
    recieve_data = connect.read(12).decode() # чтение строки из 24 символов в строку
    check_connect(connect)
    return recieve_data

def openconnect(port, speed):
    connect = serial.Serial(port, speed)
    time.sleep(1)
    while not connect.is_open:
        openconnect(port, speed)
    is_connected = False
    while not is_connected:
        print("Waiting for arduino...")
        connect.write(start_connect.encode())
        connect_flag = connect.read(1).decode()
        check_connect(connect)
        if not connect_flag:
            time.sleep(0.1)
            continue
        if connect_flag == 'r':
            is_connected = True
            print('Connected!')
    return connect

def closeconnect(connect):
    connect.close()
    
def process_data(data): # разбиваем строку на отдельные значения 
    data = data.split(';')
##    print(data)
    linearVelocityRight = float(data[0])
    linearVelocityLeft = float(data[1])
    return linearVelocityRight, linearVelocityLeft
    


    


