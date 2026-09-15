// RPM-Test: Funduino TT-Getriebemotor mit Hall-Encoder (F23105692)
// Pinbelegung lt. Datenblatt: G=GND, V=3.3-5V, H1/H2=Encoder, M+/M-=Motor

const byte PIN_H1 = 2;   // Interrupt-fähiger Pin
const byte PIN_H2 = 3;

volatile long pulseCount = 0;

// Pulse pro Motorumdrehung (VOR dem Getriebe) - Datenblatt nennt keinen
// genauen Wert, deshalb hier kalibrieren (Motor 1x von Hand drehen,
// pulseCount zählen -> Wert eintragen).
const float PULSES_PER_MOTOR_REV = 8.0;
const float GEAR_RATIO = 48.0;   // lt. Datenblatt 1:48

const unsigned long INTERVAL_MS = 200;
unsigned long lastTime = 0;

void onPulse() {
  pulseCount++;
}

void setup() {
  Serial.begin(9600);
  pinMode(PIN_H1, INPUT);
  pinMode(PIN_H2, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_H1), onPulse, RISING);
  lastTime = millis();
}

void loop() {
  unsigned long now = millis();
  if (now - lastTime >= INTERVAL_MS) {
    noInterrupts();
    long count = pulseCount;
    pulseCount = 0;
    interrupts();

    // Drehrichtung: Phasenlage H2 relativ zur H1-Flanke
    bool dir = digitalRead(PIN_H2);

    float motorRevsPerMin = (count / PULSES_PER_MOTOR_REV) * (60000.0 / INTERVAL_MS);
    float outputRpm = motorRevsPerMin / GEAR_RATIO;   // Abtriebsdrehzahl

    Serial.print("Abtrieb RPM: ");
    Serial.print(outputRpm, 1);
    Serial.print(" | Richtung: ");
    Serial.println(dir ? "CW" : "CCW");

    lastTime = now;
  }
}