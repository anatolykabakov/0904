import socket

class TaskManager(object):
    def __init__(self, tracking, controller, gridmap, host=None, port=None):
        self.controller = controller
        self.tracking = tracking
        self.gridmap = gridmap
        self.task = ''
        self.run = False
        self.done = False
        self.cnc_command = ''
        self.prev_cnc_command = ''
        
        # Create a TCP/IP socket
        self.sock = None

        self.host = host
        self.port = port

        #self.server_address = (host, port)
        
    def connect2server(self):
        # Create a TCP/IP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        # Connect the socket to the port where the server is listening
        print('connecting to {} port {}'.format(*self.server_address))
        self.sock.connect(self.server_address)
        
        
    def setTask(self, cnc_command):

        cnc = cnc_command.split(' ')
        self.task = cnc[0]
        if self.task == 'move2trajectory':
            self.run = True
            self.done = False
            self.tracking.trajectory = []
            self.tracking.i = 0
            self.controller.error_pos=1
            for point in cnc[1:]:
                point = point.split(',')
                x = point[0]
                y = point[1]
                self.tracking.trajectory.append([float(x), float(y)])
                
        if self.task == 'move2point':
            self.run = True
            self.done = False
            self.controller.error_pos = 1
            self.tracking.x_goal = float(cnc[1])
            self.tracking.y_goal = float(cnc[2])

        if self.task == 'movevelocity':
            for point in cnc[1:]:
                point = point.split(',')
                linear_velocity = float(point[0])
                angular_velocity = float(point[1])
##                print(str(speed_right)+' '+str(speed_left))
                self.controller.linear_velocity = linear_velocity
                self.controller.angular_velocity  = angular_velocity
                
    def send_data(self, command, map_array=None):
        # Send data

        if command == 'm':#send map
            print('send')
            self.sock.send(command.encode())
            self.sock.send(map_array.tobytes())
        if command == 'g':#send map
            self.sock.send('g'.encode())
        if command == 'd':#send map
            self.sock.send('g'.encode())

    def recv_data(self):
        data = self.sock.recv(1)
        if data:
            if data.decode() == 'p':# move to point
                point = self.sock.recv(3)
                point = point.decode()
                self.cnc_command = 'move2point' + ' ' + point
            if data.decode() == 'v':# move velocity v omega
                point = self.sock.recv(7)
                velocity = point.decode()
                self.cnc_command = 'movevelocity' + ' ' + velocity
            if data.decode() == 't':# move to trajectory
                trajectory = []
                len_messsage = self.sock.recv(2)
                len_messsage = len_messsage.decode()
                recv_traj = self.sock.recv(int(len_messsage))
                recv_traj = recv_traj.decode()
                self.cnc_command = 'move2trajectory' + ' ' + recv_traj
                print(self.cnc_command)
            if data.decode() == 'm':#send map
                print('send')
                command = 'm'
                self.sock.send(command.encode())
                self.sock.send(self.gridmap.data.tobytes())
            

    def update(self):
        ''' TO DO:
            написать обновление задач для робота
            пользователь должен задавать задание (ехать к точке или по траектории)
            и задавать параметры (целевую точку или траекторию)

            формат сообщения "команда(move to pose) задание(x_goal, y_goal)"

            пример:
            
            move2trajectory 1,0 1,1 0,1 0,0
            move2point 1 1

            обмен сообщений по TCP, WIFI сеть
            
        '''

        #self.recv_data()
##        if self.done == False:
##            self.send_data(command='g')

##        self.send_data(command='m', map_array=self.gridmap.data)
        
        if self.cnc_command != self.prev_cnc_command:
##            print('cnc: '+str(self.cnc_command)+' prev cnc: '+str(self.prev_cnc_command))
            self.setTask(self.cnc_command)
            self.prev_cnc_command = self.cnc_command

        if self.tracking.stop == True:
            self.done = True
            self.run = False
##            self.tracking.x_goal = 0
##            self.tracking.y_goal = 0
##            self.tracking.trajectory = []
            self.controller.speed_right = 0
            self.controller.speed_left  = 0
            self.controller.linear_velocity = 0
            self.controller.angular_velocity = 0
            

        
