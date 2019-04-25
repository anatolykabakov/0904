import math
from neato_api import xv21
BASE_WIDTH = 248    # millimeters
MAX_SPEED = 300     # millimeters/second
def potencial_field(xr, yr, xg, yg, Vu, scan):
    '''
    xr - x координата робота в метрах
    yr - y координата робота в метрах
    xg - x координата цели в метрах
    yg - y координата цели в метрах
    Vu - желаемая скорость движения
    scan - массив точек лидара
    '''
    rd = 2 # дистанция рекции
    Ka = 1.0 # коэффициент притяжения
    Kr = 2.0 # коэффициент отталкивания
    dg = math.sqrt((xg-xr)**2+(yg-yr)**2)#расстояние до цели
    if dg <= 0.1:
        stop = True
    else:
        stop = False
    Xrf = []
    Yrf = []
    test_dist = []

    #Цикл по всем измерениям лидара
    for point in scan:
        alpha = point[0] #угол измерения лидара градусы
        dist  = point[1] #дистанция измерения лидара миллиметры
        alpha = alpha*math.pi/180 #градусы в радианты
        #alpha = alpha - 2*math.pi
        dist  = dist/1000 # миллиметры в метры
        
        if (0<=alpha) and (alpha<=math.pi/2):
            gamma = -alpha
        if (3*math.pi/2 <= alpha) and (alpha<= 2*math.pi):
            gamma = 2*math.pi- alpha
        per = -1*math.pi/2
        # Множество отталкивания
        if ((dist <= rd) and  ((0<=alpha<=math.pi/2) or (3*math.pi/2<=alpha<=2*math.pi)) ):
            f = ((rd - dist)/rd)**2
            Xrf.append(f*math.cos(gamma))
            Yrf.append(f*math.sin(gamma))
            test_dist.append(round(dist,1))
        
    #Общий вектор отталкивания
    #for dist in test_dist:
    print(test_dist)
    if len(Xrf)>0:
        RF = [sum(Xrf)/len(Xrf), sum(Yrf)/len(Yrf)]
    else:
        RF=[0,0]
    print(RF)
    
    #Вектор притяжения
    Xra = (xg-xr)/dg
    Yra = (yg-yr)/dg
    RA = [Xra,Yra]
    print(RA)
    #Вектор движения
    Xrf = RF[0]
    Yrf = RF[1]
    RV_x = Xra*Ka-Xrf*Kr
    RV_y = Yra*Ka-Yrf*Kr
    print(RV_x, RV_y)
    LinearVelocity = Vu*math.sqrt(RV_x**2+RV_y**2)
    #AngularVelocity = Vu*2*math.atan2(RV_y, RV_x)/0.1875/math.pi
    AngularVelocity = 3*math.atan2(RV_y, RV_x)
    return LinearVelocity, AngularVelocity,stop

class Neato(object):
    def __init__(self, port):
        self.api = xv21(port)
        self.last_encoders = []
        self.encoders_current = []
        self.scan
        self.x = 0
        self.y = 0
        self.th = 0
        self.prev_time = time.time()
        self.current_time = time.time()
        self.dt = 0
        self.lvel = 0
        self.avel = 0

    def sense(self):
        self.encoders_current = self.api.getMotors()
        self.scan     = self.api.getScan()

    def update_state(self):
        self.current_time = time.time()
        self.dt = self.current_time - self.prev_time
        self.prev_time = self.current_time
        left=self.encoders_current[1]
        right=self.encoders_current[0]
        d_left = (left - self.last_encoders[0])/1000.0
        d_right = (right - self.last_encoders[1])/1000.0
        self.last_encoders = [left, right]
        
        dx = (d_left+d_right)/2
        dth = (d_right-d_left)/(BASE_WIDTH/1000.0)

        x = math.cos(dth)*dx
        y = -math.sin(dth)*dx
        self.x += math.cos(self.th)*x - math.sin(self.th)*y
        self.y += math.sin(self.th)*x + math.cos(self.th)*y
        self.th += dth
        self.lvel = dx/self.dt
        self.avel = dth/self.dt

    def drive(self, lvel, avel):
        vl = (2*lvel - avel*(BASE_WIDTH/1000))/2
        vr = (2*lvel - avel*(BASE_WIDTH/1000))/2
        self.api.setMotors(100,100,100)

    def stop(self):
        self.api.stop()
        

if __nama__ == '__main__':
    robot = Neato('/dev/ttyACM1')
    Vu = 0.2
    xg,yg=30,0
    while True:
        try:
            robot.sense()
            robot.update_state()
            LinearVelocity, AngularVelocity,stop = potencial_field(robot.x, robot.y, xg, yg, Vu, robot.scan)
            robot.drive(LinearVelocity, AngularVelocity)

        except KeyboardInterrupt:
            robot.stop()
