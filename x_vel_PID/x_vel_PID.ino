#include "KalmanMPU6050.h"
unsigned long last_Time = 0;
float dt;
float last_ER;

float Kp_speed = 0.001;
float Ki_speed = 0;
float Kd_speed = 0;

const int IN1 = 6;// Motor setup 
const int IN2 = 9;

const int IN3 = 10;  
const int IN4 = 11;

int applyDeadzone(int16_t pidOut, int minPWM = 75) 
{
  if (pidOut == 0) return 0;
  int mapped = map(abs(pidOut), 0, 255, minPWM, 255);
  return (pidOut > 0) ? mapped : -mapped;
}

void setup() 
{
  Serial.begin(9600);
  IMU::init();
  IMU::read();


  pinMode(IN1,OUTPUT); // steup Motors
  pinMode(IN2,OUTPUT);
    
  pinMode(IN3,OUTPUT);
  pinMode(IN4,OUTPUT);
  
  
  last_Time = millis();
  delay(5000);
}

void loop() 
{
  IMU::read();
  //Serial.println("VX:" +String(getVX(IMU::getPitch(),IMU::getRawAccelX())));
  float Vx = getVX(IMU::getPitch(),IMU::getRawAccelX());
  float output = speed_PID(error(Vx));
  set_Speed(applyDeadzone(output));
  delay(100);
}


float getVX(float pitch, float ares)
{
  // get velocity through integral 
  unsigned long now = millis();

  dt = (now - last_Time) / 1000.0f;
  ares= (float)ares / 16384.0f * 9.81f;
  float ax = ares * cos(pitch);
  if( abs(ax) < 0.04 ) ax=0;

  Serial.println("dt:"+String(dt));
  Serial.println("ax:"+String(ax));
  Serial.println("ares:"+String(ares));
  Serial.println("pitch:"+String(pitch));

  last_Time = now;
  static float Vx;
  Vx += ax * dt;
  Serial.println("Vx:"+String(Vx));
  return Vx;
}

float error(float Vx)
{
  return -Vx;
}

float speed_PID (float ER)
{
  float Derivative = (ER - last_ER) / dt ;

  float integral = ER * dt;
  integral = constrain(integral, -100 , 100);

  float output = Kp_speed * ER + Ki_speed * integral + Kd_speed * Derivative;
  last_ER = ER;
  output = constrain(output, -255 , 255);
  return output;

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

