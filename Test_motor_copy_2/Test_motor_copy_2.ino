
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
  digitalWrite(IN1,HIGH);
  digitalWrite(IN3,HIGH);
  digitalWrite(IN2,LOW);
  digitalWrite(IN4,LOW);
  delay(2000);
  digitalWrite(IN1,LOW);
  digitalWrite(IN3,LOW);
  digitalWrite(IN2,HIGH);
  digitalWrite(IN4,HIGH);
  delay(2000);
  
  
}
