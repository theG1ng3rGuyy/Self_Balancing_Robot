#include "KalmanMPU6050.h"
unsigned long last_Time = 0;
float dt;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  IMU::init();
  IMU::read();
  last_Time = millis();
}

void loop() {
  // put your main code here, to run repeatedly:
  IMU::read();
  Serial.println("VX:" +String(getVX(IMU::getPitch(),IMU::getRawAccelX())));
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
