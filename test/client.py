import socket
import sys
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as colormap
import png


array = np.zeros((100,100), dtype=np.uint8)
command = 'm'
x = 0
y = 0

# Create a TCP/IP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Connect the socket to the port where the server is listening
server_address = ('localhost', 7777)
print('connecting to {} port {}'.format(*server_address))
sock.connect(server_address)


def send_data(sock, command, map_array=None):
    # Send data

    if command == 'm':#send map
        sock.send(command.encode())
        sock.send(map_array.tobytes())

def recv_data(data):
    if data.decode() == 'p':# move to point
            point = sock.recv(3)
            point = point.decode()
            point = point.split(',')
            x = float(point[0])
            y = float(point[1])
            print('x: {0}, y: {1}'.format(x, y))
    if data.decode() == 'v':# move velocity v omega
        point = sock.recv(7)
        point = point.decode()
        point = point.split(',')
        v = float(point[0])
        w = float(point[1])
        print('linear velocity: {0}, angilar velocity: {1}'.format(v, w))
    if data.decode() == 't':# move to trajectory
        trajectory = []
        len_messsage = sock.recv(2)
        len_messsage = len_messsage.decode()
        recv_traj = sock.recv(int(len_messsage))
        recv_traj = recv_traj.decode()
        recv_traj = recv_traj.split(' ')
        for point in recv_traj:
            point = point.split(',')
            x = float(point[0])
            y = float(point[1])
            trajectory.append([x, y])
        print(trajectory)
    if data.decode() == 'm':# get map
        send_data(sock, command=command, map_array=array)
try:
    while True:
        data = sock.recv(1)
        if data:
            recv_data(data)
        print(data.decode())

        x += 1
        y += 1
        array[x][y] = 1
finally:
    print('closing socket')
    sock.close()
