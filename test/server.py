import socket
import sys
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as colormap
import time

##command = 'p'#'move2point'
command = 't'#'move2trajectory'
##command = 'v'#'move2velocity'
##command = 'm'#get map
trajectory = '6,5 6,6 5,6 5,5'
point = '1,1'
velocity = '0.5,1.0'


def send_data(sock, command, point=None, velocity=None, trajectory=None):
    # Send data
    if command == 'p':#move2point
        print(command)
        sock.send(command.encode())
        sock.send(point.encode())
    if command == 'v':#move velocity
        sock.send(command.encode())
        sock.send(velocity.encode())
    if command == 't':#move2trajectory
        sock.send(command.encode())
        sock.send(str(len(trajectory)).encode())
        sock.send(trajectory.encode())
    if command == 'm':#get map
        sock.send(command.encode())
    if command == 'g':#get map
        sock.send('g'.encode())





# Create a TCP/IP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Bind the socket to the port
server_address = ('', 7777)
print('starting up on {} port {}'.format(*server_address))
sock.bind(server_address)

# Listen for incoming connections
sock.listen(1)
# Wait for a connection
print('waiting for a connection')
connection, client_address = sock.accept()
print('connection from', client_address)
#send move
#send_data(connection, command=command, point=None, velocity=None, trajectory=trajectory)
try:
    while True:
        #get map
        send_data(connection, command=command, point=None, velocity=None, trajectory=trajectory)
        #send_data(connection, command='m', point=None, velocity=None, trajectory=None)
        '''data = connection.recv(1)
        if data.decode() == 'm':# get map
            time.sleep(0.1)
            mapbytes = connection.recv(80000)
            mapimg = np.reshape(np.frombuffer(mapbytes, dtype=np.uint8), (100, 100))
            plt.imshow(mapimg, cmap=colormap.gray, origin='lower')
            plt.pause(0.001)
##        if data.decode() == 'd':# get map
##            send_data(connection, command='m', point=None, velocity=None, trajectory=None)
'''

finally:
    # Clean up the connection
    connection.close()
