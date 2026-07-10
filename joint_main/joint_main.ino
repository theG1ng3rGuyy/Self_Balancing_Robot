#include "KalmanMPU6050.h"
long then = 0; //cehck for other time variabels


float ER;        // eror
float setpoint = 0;  // the value that should be reached steppoiont
float MS;        // real value measure
int16_t output;
const int16_t MAX_OUTPUT = 255;

float Kp = 2;
float Ki = 0.1;
float Kd = 0;

float derivattive;
float Integral;

const int IN1 = 6;// Motor setup 
const int IN2 = 9;

const int IN3 = 10;  
const int IN4 = 11;


unsigned long last_Time = 0;  //
float dt;
float lastError;

float Kalman_Output;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  pinMode(IN1,OUTPUT); // steup Motors
  pinMode(IN2,OUTPUT);
    
  pinMode(IN3,OUTPUT);
  pinMode(IN4,OUTPUT);

  IMU::init();
  IMU::read();

  //pinMode(LED_BUILTIN, OUTPUT);

}

void loop() 
{

  output = PID_Motor(error());
  set_Speed(output);// put your main code here, to run repeatedly:
}


////////////////////////////////////////////////////////

float PID_Motor(float ER) {
  unsigned long now = millis();             // Zeit seit Programstart
  dt = (float)(now - last_Time) / 1000.0f;  // dt berechhnen in Sekunden
  last_Time = now;                          //eventuell millis();
 // Serial.println(dt);                       // debug


  Integral += ER * dt;  // kumulierte summe
  Integral = constrain(Integral, -100, 100); // gains

  derivattive = (ER - lastError) / dt;
  lastError = ER;

  output = ER * Kp + Integral * Ki + derivattive * Kd;

  return (int16_t) constrain(output,-255,255);// antiwindup (clamping) and 
}


float error()
{
  IMU::read(); //why
  Serial.println(IMU::getPitch());
  ER = setpoint - IMU::getPitch(); // noch nicht  fertig 
  Serial.println("Error:"+String(ER));
  return ER;
}

void set_Speed(int speed){
  Serial.println("Speed:"+String(speed));
  if (speed >= 0){
    analogWrite(IN1,speed); // teste wellcher richtung 
    analogWrite(IN3,speed);
    digitalWrite(IN4,LOW);
    digitalWrite(IN2,LOW);
  }
  else
  {
  speed = abs(speed);
  analogWrite(IN2,speed);
  analogWrite(IN4,speed);
  digitalWrite(IN1,LOW);
  digitalWrite(IN3,LOW);  
  }
}
