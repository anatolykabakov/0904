import math
from utils import normalize_angle


class Controller(object):
    def __init__(self, robot, linear_velocity, angular_velocity, target_speed, max_angular_velocity):
        self.robot = robot
        self.linear_velocity  = linear_velocity
        self.angular_velocity = angular_velocity
        self.error_pos = 0
        self.speed_right = 0
        self.speed_left = 0
        self.error_orientation = 0
        self.error_pos = 0
        self.dist_right = 0
        self.dist_left = 0
        self.x_goal = 0
        self.y_goal = 0
        self.trajectory = []
        self.target_speed = target_speed
        self.max_angular_velocity = max_angular_velocity
        self.speed_up = None
        self.omegaRight = 0
        self.omegaLeft = 0
        self.v = 0
        self.omega = 0
        
        

    def orientation_controller(self):
        x_diff = self.x_goal - self.robot.x
        y_diff = self.y_goal - self.robot.y
        self.error_pos = math.sqrt(x_diff**2 + y_diff**2)
        self.error_orientation = normalize_angle(math.atan2(y_diff, x_diff) - (self.robot.yaw))
##        print('goal: ' + str(math.atan2(y_diff, x_diff))+' - theta: '+str(self.robot.yaw)+' = '+str(self.error_orientation))

    def speed_controller(self):
        if self.error_orientation == 0:
            self.angular_velocity = 0
        else:
            self.angular_velocity = self.error_orientation#/ self.robot.delta_time
##        print(self.angular_velocity)
        if self.speed_up == True:
            self.linear_velocity = self.target_speed
##            self.linear_velocity += self.speed_control_up(self.target_speed, self.robot.v)
            
        else:
            self.linear_velocity = (self.error_pos)
##        print(self.linear_velocity)
        #self.control_move()
        
            
        self.restrictions_speed(self.target_speed, self.max_angular_velocity)
        
        
##        self.speed_right = ((2*self.linear_velocity)+(self.angular_velocity*self.robot.length))/2# m/sec
##        self.speed_left  = ((2*self.linear_velocity)-(self.angular_velocity*self.robot.length))/2
##
##        self.omegaRight = self.speed_right/self.robot.radius#   mm/s / mm =  rad за сек
##        self.omegaLeft  = self.speed_left /self.robot.radius#
##
##            
##        # фактическая угловая скорость поворота робота
##        self.omega = (self.robot.radius/self.robot.length)*(self.omegaRight - self.omegaLeft)
##
##
##        self.dist_right = self.speed_right*self.robot.delta_time
##        self.dist_left = self.speed_left*self.robot.delta_time
        
    def restrictions_speed(self, max_linear_speed, max_angular_speed):
        if self.linear_velocity > max_linear_speed:
            self.linear_velocity = max_linear_speed
        if self.linear_velocity <  -max_linear_speed:
            self.linear_velocity = -max_linear_speed
        if self.angular_velocity > max_angular_speed:
            self.angular_velocity = max_angular_speed
        if self.angular_velocity <  -max_angular_speed:
            self.angular_velocity = -max_angular_speed
            
    def control_move(self):
        if abs(self.error_orientation) > 0.15:
            self.linear_velocity = 0

    def speed_control_up(self, target, current):
        Kp = 0.5
        return Kp*(target - current)
            
    def update(self):
        self.orientation_controller()
        self.speed_controller()
        

        
