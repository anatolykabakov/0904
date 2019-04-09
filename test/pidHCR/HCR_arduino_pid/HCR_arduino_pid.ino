//Использование выводов
/*
===PWM==============================================================
Моторы
4 ---> правый мотор PWM1 (+ земля)
5 ---> левый мотор PWM2


===DIGITAL==========================================================
Моторы
52 ---> Direction-пин правого мотора DIR1
53 ---> Direction-пин левого мотора DIR2



===ANALOG==========================================================

нет
*/

#include <Metro.h>

#include <Math.h>
#include <TimerOne.h> // http://www.arduino.cc/playground/Code/Timer1
#include <Wire.h>

// #include <Check_times.cpp>


// MegaADK DIGITAL PINS USABLE FOR INTERRUPTS 2, 3, 18, 19, 20, 21
//                                                 I2C pins 20, 21

//Encoder variables

const byte encoderRpinA = 2;                              //A pin -> the interrupt pin (2)
const byte encoderRpinB = 17;                              //B pin -> the digital pin (16)
const byte encoderLpinA = 3;                              //A pin -> the interrupt pin (3)
const byte encoderLpinB = 16;                              //B pin -> the digital pin (17)


byte encoderRPinALast;
byte encoderLPinALast;
double wheelSpeedR = 0;  // Скорость правого колеса с энкодера
double wheelSpeedL = 0;  // Скорость левого колеса с энкодера
unsigned long wheelImpR = 0; // число импульсов с энкодера правого колеса 
unsigned long wheelImpL = 0; // число импульсов с энкодера левого колеса 

//PID variables
double outputSumLeft = 0;
double outputSumRight = 0;
double Kp = 1;
double Ki = 0.0;
double P_Left = 0;
double P_Right = 0;
double I_left = 0;
double I_right = 0;
double errorLeft = 0;
double errorRight = 0;
double outPidLeft = 0;
double outPidRight = 0;
unsigned long prev_timeL = 0;
unsigned long prev_timeR = 0;


// Timer variables
const long Timer1Interval=50000;                                // 50 ms = 50 times per sec - Timer interrupt interval
double dT = double(Timer1Interval)/1000000;           // 50 ms период счета 
unsigned long prevtime = 0;

//Motor control variables
const int MotorRdir = 52;    //Right motor Direction Control pin
const int MotorLdir = 53;    //Left motor Direction Control pin
const int MotorRpwm = 4;     //Right motor PWM Speed Control pin
const int MotorLpwm = 5;     //Left motor PWM Speed Control pin
double SetSpeedR = 0;   //Wish Speed of right motor
double SetSpeedL = 0;   //Wish Speed of left motor
bool DirectionR = 0;     //Right Motor Direction
bool DirectionL = 0;     //Left Motor Direction

//-------------------------События------------------------------------------------------


//------------------------------------------------
#include <string.h>
char buffer[8];
double LinearVelocity = 0.0; 
double AngularVelocity = 0;
double prevLinearVelocity = 0.0; 
double prevAngularVelocity = 0;
//double AngularVelocity ;
double readV, readW;
double wheelLeftS = 0;
double wheelRightS = 0;
double wheelLeftV = 0;
double wheelRightV = 0;
double omegaRight = 0;
double omegaLeft = 0;
double R = 0.0682;
double L = 0.275;
double V = 0;
double omega = 0;
double Vl = 0;
double Vr = 0;
double SetV = 0;
double SetW = 0;
double maxSpeed = 0.94; // максимальная линейная скорость при скважности 100%, в м/с
double maxSpeedR = 0.67;
double maxSpeedL = 0.94;

double yaw = 0;
double x = 0;
double y = 0;

bool printflag = false;
bool is_connected = false;
bool time_over = false;
int rightPWM = 0;
int leftPWM = 0;




unsigned long time;


void setup() {
   Init();
}
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Главный цикл ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ 
void loop() { 
  // --------------- Чтение уставок по линейной и угловой скорости --------------------
  get_messages_from_Serial1();
  check_time();
 }
 //loop ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void check_time()
{
  if (time_over)
  {
    time_over = false;
    SetSpeed(LinearVelocity, AngularVelocity);
    Movement_noPID(SetSpeedR,SetSpeedL);
  }
}
void reset_var(){
  x = 0;
  y = 0;
  yaw = 0;
  V = 0;
  SetSpeedR = 0;   //M/S - линейная скорость колеса
  SetSpeedL = 0;
}
void Init(){
  Wire.begin();
  Serial1.begin(57600);//Initialize the Serial1 port
  while (!Serial1) ; // while the Serial1 stream is not open, do nothing
  MotorsInit();
  EncoderInit();//Initialize encoder 
}


void MotorsInit() { //Initialize motors variables 
  DirectionR = LOW;
  DirectionL = LOW;
  SetSpeedR = 0;
  SetSpeedL = 0;
  
  pinMode(MotorRdir, OUTPUT);
  pinMode(MotorLdir, OUTPUT);
  pinMode(MotorRpwm, OUTPUT);
  pinMode(MotorLpwm, OUTPUT);
  
  digitalWrite (MotorRdir, DirectionR);
  digitalWrite (MotorLdir, DirectionL);
  analogWrite (MotorRpwm, SetSpeedR);
  analogWrite (MotorLpwm, SetSpeedL);
}
void EncoderInit() { //Initialize encoder interruption 

  pinMode(encoderRpinA,INPUT);  // Right weel
  pinMode(encoderRpinB,INPUT);  
  pinMode(encoderLpinA,INPUT);  // Left weel
  pinMode(encoderLpinB,INPUT);  
 
  // Привязка прерывания по импульсу энкодера
  attachInterrupt(digitalPinToInterrupt(encoderRpinA), WheelPulseR, RISING ); // вызов процедуры по прерыванию. Параметры: номер прерывания (не ножки), имя процедуры, состояние сигнала
  attachInterrupt(digitalPinToInterrupt(encoderLpinA), WheelPulseL, RISING );  // ЗАМЕНА, была ссылка на DecodeSpeedL
  
  // Настройка таймера
  Timer1.initialize(Timer1Interval);
  Timer1.attachInterrupt(Timer_finish);
 
}
void WheelPulseR(){   // Счетчик спиц правого колеса 
  wheelImpR ++;
}
void WheelPulseL(){   // Счетчик спиц левого колеса 
  wheelImpL ++;
}
void Timer_finish()  {
  

  wheelSpeedR = double(wheelImpR / dT); // число импульсов за сек
  wheelSpeedL = double(wheelImpL / dT); // число импульсов за сек

  // пройденный колесом путь, м
  wheelRightS = ((wheelSpeedR / 663) * 2 * 3.14 *  R) ; // метры L = 2*PI*R*n/N 
  wheelLeftS  = ((wheelSpeedL / 663) * 2 * 3.14 *  R); //* 
  
  // линейная скорость колеса
  wheelRightV = wheelRightS/ 1; // mетры за сек
  wheelLeftV  = wheelLeftS / 1;

  // угловая скорость колеса
  omegaRight = wheelRightV/R;   // rad за сек
  omegaLeft  = wheelLeftV/R;

  if (DirectionR) {omegaRight = -1*omegaRight;}
  if (DirectionL) {omegaRight = -1*omegaLeft;}
  
  // фактическая линейная скорость центра робота
  V     = (R/2)*(omegaRight + omegaLeft);//m/s
  // фактическая угловая скорость поворота робота
  omega = (R/L)*(omegaRight - omegaLeft);

  yaw+=(omega * dT);    // направление в рад
  
  yaw = normalize_angle(yaw);
  x += V*cos(yaw) * dT; // в метрах
  y += V*sin(yaw) * dT;
  

  // проверка
  Vr = (((2*V)+(omega*L))/(2*R))*R; //M/S
  Vl = (((2*V)-(omega*L))/(2*R))*R;
  
  wheelImpR = 0;
  wheelImpL = 0;
  time_over = true;
}

double normalize_angle(double angle)
{
  while(angle > PI)
  {
    angle -= 2.0 * PI;
  }
  while(angle < -PI)
  {
    angle += 2.0 * PI;
  }
  return angle;
}
void SetSpeed(double LinearVelocity, double AngularVelocity){ 
  SetSpeedR = ((2*LinearVelocity)+(AngularVelocity*L))/2;   //M/S - линейная скорость колеса
  SetSpeedL = ((2*LinearVelocity)-(AngularVelocity*L))/2;
}
void PID_Controller()
{
  /*
  error = desired_value – actual_value
  integral = integral + (error*iteration_time)
  derivative = (error – error_prior)/iteration_time
  output = KP*error + KI*integral + KD*derivative + bias
  error_prior = error
  sleep(iteration_time)
  */
}

  
}
void Movement_noPID(double rightVelOut,double leftVelOut){//move
  // Проверка направления вращения
  if (rightVelOut< 0) {rightVelOut = abs(rightVelOut); DirectionR = HIGH;}
  else {DirectionR = LOW;}
  if (leftVelOut < 0) {leftVelOut = abs(leftVelOut);   DirectionL = HIGH;}
  else {DirectionL = LOW;}
  // Переход от метров в сек к ШИМ 0..255
  rightPWM= int(rightVelOut * 255 /maxSpeed); // уставка скорости
  leftPWM = int(leftVelOut * 255 /maxSpeed);
  // Ограничения 13..255
  if (rightPWM < 13) {rightPWM = 0;}
  if (leftPWM < 13) {leftPWM = 0;}
  if (rightPWM >= 255) {rightPWM = 255;}
  if (leftPWM >= 255) {leftPWM = 255;}
  // Подаем на двигатели ШИМ 13..255
  analogWrite (MotorRpwm,rightPWM);      //motor1 move forward at speed a
  digitalWrite(MotorRdir,DirectionR);  
  analogWrite (MotorLpwm,leftPWM);      //motor2 move forward at speed b
  digitalWrite(MotorLdir,DirectionL);  
}

void get_messages_from_Serial1()
{
  if(Serial1.available() > 0)
  {
    // The first byte received is the instruction
    int order_received = Serial1.read();

    if(order_received == 's')
    {
      // If the cards haven't say hello, check the connection
      if(!is_connected)
      {
        is_connected = true;
        Serial1.print("r");
      }
    }
    else
    {
      switch(order_received)
      {

        case 'v'://если v, то считываем уставку по скорости
        {

          String line = Serial1.readStringUntil('\n');// считываем скорости для левого и правого колеса [40 50]
          line.toCharArray(buffer,10);//переводим в char
          LinearVelocity        = atof(strtok(buffer," "));//разделяем на скорости левого и правого колеса
          AngularVelocity       = atof(strtok(NULL,  " "));
          

          break;
        }
        case 'd'://если d, то печатаем текущие значения
        {
          if(V>=0){Serial1.print ("+");Serial1.print (V); Serial1.print (";");}
          else {Serial1.print (V); Serial1.print (";");}
          if(yaw>=0){Serial1.print ("+");Serial1.print (yaw); Serial1.print (";");}
          else {Serial1.print (yaw); Serial1.print (";");}
          if(x>=0){Serial1.print ("+");Serial1.print (x); Serial1.print (";");}
          else {Serial1.print (x); Serial1.print (";");}
          if(y>=0){Serial1.print ("+");Serial1.print (y); Serial1.print (";");}
          else {Serial1.print (y); Serial1.print (";");}
          
          break;
        }
        // Unknown order
        default:

          return;
      }
    }
    Serial1.print("c");
 
  }
}
      
