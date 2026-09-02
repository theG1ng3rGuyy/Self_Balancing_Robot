#include "KalmanMPU6050.h"



// Prototypen (manuell, da Auto-Generierung bei Default-Parametern manchmal versagt)
int applyDeadzone(int16_t pidOut, int minPWM = 75);
float PID_Motor(float ER);
float error();
void set_Speed(int speed);


long then = 0; //cehck for other time variabels



float ER;        // eror
float setpoint = 0;  // the value that should be reached steppoiont
float MS;        // real value measure
int16_t output;
const int16_t MAX_OUTPUT = 255;

float Kp = 4;
float Ki = 0;
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
   last_Time = millis();

  //pinMode(LED_BUILTIN, OUTPUT);

}

void loop() 
{

  output = PID_Motor(error());
  set_Speed(applyDeadzone(output));// put your main code here, to run repeatedly:
  delay(10);
}


////////////////////////////////////////////////////////

//Hier die komplette PID_Motor()-Funktion mit dem Filter integriert, damit die genaue Stelle klar ist:

float filteredD = 0;  // NEU: global deklarieren, z.B. bei den anderen globalen Variablen oben im Code
float alpha = 0.15;   // NEU: global, Glättungsfaktor

float PID_Motor(float ER) {
  unsigned long now = millis();
  dt = (float)(now - last_Time) / 1000.0f;
  if (dt < 0.001f) dt = 0.001f;
  last_Time = now;

  Integral += ER * dt;
  Integral = constrain(Integral, -100, 100);

  // ALT (raus):
  // derivattive = (ER - lastError) / dt;

  // NEU (rein):
  float rawD = (ER - lastError) / dt;
  filteredD = alpha * rawD + (1 - alpha) * filteredD;

  lastError = ER;

  // ALT: output = ER * Kp + Integral * Ki + derivattive * Kd;
  output = ER * Kp + Integral * Ki + filteredD * Kd;  // NEU: filteredD statt derivattive

  return (int16_t) constrain(output, -255, 255);
}


float error()
{
  IMU::read(); //why
  
  //Serial.println(IMU::getPitch());
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

int applyDeadzone(int16_t pidOut, int minPWM = 75) {
  if (pidOut == 0) return 0;
  int mapped = map(abs(pidOut), 0, 255, minPWM, 255);
  return (pidOut > 0) ? mapped : -mapped;
}
