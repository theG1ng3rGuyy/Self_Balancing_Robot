
const int IN1 = 6;
const int IN2 = 9;

const int IN3 = 10;  
const int IN4 = 11;


void setup() {
  // put your setup code here, to run once:
    pinMode(IN1,OUTPUT);
    pinMode(IN2,OUTPUT);
    
    
    pinMode(IN3,OUTPUT);
    pinMode(IN4,OUTPUT);


    //test Motor 



}

void loop() {
  analogWrite(IN1,75);
  analogWrite(IN3,75);
  analogWrite(IN2,0);
  analogWrite(IN4,0);
  delay(2000);

  
  
}
