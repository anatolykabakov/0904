



class MotionControl(object):
    def __init__(self, robot, controller):
        self.robot = robot
        self.controller = controller
        self.point = []
        self.i = 0
        self.x_goal = 0
        self.y_goal = 0
        self.trajectory = []
        self.stop = False


    def move_to_point(self, x_goal, y_goal):
        self.controller.x_goal = x_goal
        self.controller.y_goal = y_goal
        if self.controller.error_pos<0.1:
            self.stop = True
        
    def move_to_trajectory(self, trajectory):
        if trajectory[len(trajectory)-1] != trajectory[self.i]:
            if abs(self.controller.error_pos)>0.1:
                point = trajectory[self.i]
                self.controller.x_goal = point[0]
                self.controller.y_goal = point[1]
                self.controller.speed_up = True
            else:
                self.i+=1
                self.controller.error_pos=1
                point = trajectory[self.i]
                self.controller.x_goal = point[0]
                self.controller.y_goal = point[1]
                self.controller.speed_up = True
        else:
            self.controller.speed_up = False
            if self.controller.speed_up == False and self.controller.error_pos<0.1:
                self.stop = True
                
    def update_point(self, path):
##        if abs(self.robot.x - path[-1][0]) > 0.1 or abs(self.robot.y - path[-1][1]) > 0.1:
##            self.point = path[1]
##        else:
##            self.point = [999, 999]

        if len(path)-1 > 0:
            self.point = path[1]
        else:
            self.point = path[0]

##    def with_obstacle(self, measurements):#обход препятствий для лидара
##        dist_min = 0.2#минимальное расстояние до препядствия
##        #определим есть ли препятствие 
##        obstacle_find = False
##        left = False
##        right = False
##        obstacles_left = []
##        obstacles_right = []
##        for dist, angle in measurements:
##            if dist < dist_min:
##                #определим с какой стороны препятствие
##                if math.pi/2 <= angle <= math.pi:# препятствие слева
##                    obstacle_left.append(dist)
##                    left = True
##                if 0 <= angle <= math.pi/2:# препятствие справа
##                    obstacle_right.append(dist)
##                    right = True
##                obstacle_find = True
##        #если есть препятствие
##        if obstacle_find = True:
##            #найдем максимальное отклонение среди измерений правого сектора
##            if left == True:
##                #если препятствие слева, то линейная скорость постоянна
##                minimum_left = min(obstacles_left)
##                k = -1*(dist_min - minimum_left)
##            if right == True:
##                #если препятствие справа, то линейная скорость постоянна
##                minimum_right = min(obstacles_left)
##                k = +1*(dist_min - minimum_right)
##            if left == True and right == True:
##                #если препятствие слева и справа, то линейная скорость 0
##                linear_velocity = 0
##                # определим в какую сторону двигаться - в ту, где меньше препятствий обнаружено
##                min_left  = len(obstacles_left)
##                min_right = len(obstacles_right)
##                if min_left > min_right:
##                    k = -1*(dist_min - minimum_left)
##                else:
##                    k = +1*(dist_min - minimum_right)

                
        

##    def setInitConditionTrajecrory():
##        self.tracking.trajectory = []
##        self.tracking.i = 0
##        self.controller.error_pos=1
        
