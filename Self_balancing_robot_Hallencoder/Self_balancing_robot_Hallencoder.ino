#define ENCODER_A1 2
#define ENCODER_B1 3
#define ENCODER_A2 4
#define ENCODER_B2 5

#define IN1 6
#define IN2 9
#define IN3 10
#define IN4 11

void setup() {
  Serial.begin(9600);
  
  pinMode(ENCODER_A1, INPUT);   // no pullup this time
  pinMode(ENCODER_B1, INPUT);
  pinMode(ENCODER_A2, INPUT);   // no pullup this time
  pinMode(ENCODER_B2, INPUT);
  
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void loop() {
  Serial.println("");
  Serial.print("A1: "); Serial.print(digitalRead(ENCODER_A1));
  Serial.print("  B1: "); Serial.println(digitalRead(ENCODER_B1));
  Serial.print("A2: "); Serial.print(digitalRead(ENCODER_A2));
  Serial.print("  B2: "); Serial.println(digitalRead(ENCODER_B2));
  delay(300);
}