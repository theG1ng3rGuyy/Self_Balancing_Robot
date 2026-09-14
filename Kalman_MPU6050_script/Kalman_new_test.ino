#include "KalmanMPU6050.h"

long then = 0;

void setup()
{
  Serial.begin(9600);

  IMU::init();
  IMU::read();

  pinMode(LED_BUILTIN, OUTPUT);
}

void loop()
{
  /* Reads the data from the MPU and processes it with the Kalman Filter */
  IMU::read();
    Serial.println(IMU::getPitch());

  if (millis() - then >= 1000)
  {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    then = millis();
  }
}
