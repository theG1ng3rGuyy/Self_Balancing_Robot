/*
 * MPU-6050 Kalman Filter — Y-Axis Angle
 * Fuses accelerometer + gyroscope to estimate pitch (rotation around Y axis).
 *
 * Wiring (MPU-6050 → Arduino Uno):
 *   VCC  → 3.3V  (or 5V if your module has a regulator)
 *   GND  → GND
 *   SCL  → A5
 *   SDA  → A4
 *   AD0  → GND   (I2C address = 0x68)
 *
 * Mounting: PINS UP (board flipped — Z and X axes are inverted)
 *
 * Library required: Wire.h (built-in)
 */

#include <Wire.h>

// ── MPU-6050 registers ────────────────────────────────────────────────────────
const uint8_t MPU_ADDR       = 0x68;
const uint8_t REG_PWR_MGMT   = 0x6B;
const uint8_t REG_ACCEL_XOUT = 0x3B;   // 14 bytes: AX AY AZ Temp GX GY GZ

// ── Sensor scale factors ──────────────────────────────────────────────────────
// Accelerometer: ±2 g  → 16384 LSB/g
// Gyroscope:     ±250 °/s → 131 LSB/(°/s)
const float ACCEL_SCALE = 16384.0f;
const float GYRO_SCALE  =   131.0f;

// ── Kalman filter state (for Y-axis / pitch) ──────────────────────────────────
struct Kalman {
  float angle;   // estimated angle  [°]
  float bias;    // estimated gyro bias [°/s]
  float P[2][2]; // error covariance

  // Tuning knobs
  float Q_angle;   // process noise — angle
  float Q_bias;    // process noise — gyro bias
  float R_measure; // measurement noise

  Kalman() :
    angle(0), bias(0),
    Q_angle(0.001f), Q_bias(0.003f), R_measure(0.03f)
  {
    P[0][0] = 0; P[0][1] = 0;
    P[1][0] = 0; P[1][1] = 0;
  }

  // Call every loop iteration.
  // newAngle : angle from accelerometer [°]
  // newRate  : raw gyro rate            [°/s]
  // dt       : time since last call     [s]
  float update(float newAngle, float newRate, float dt) {

    // ── 1. Predict ──────────────────────────────────────────────────────────
    float rate = newRate - bias;        // bias-corrected gyro rate
    angle += dt * rate;                 // integrate

    P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q_bias * dt;

    // ── 2. Update ───────────────────────────────────────────────────────────
    float S  = P[0][0] + R_measure;     // innovation covariance
    float K0 = P[0][0] / S;             // Kalman gain
    float K1 = P[1][0] / S;

    float y = newAngle - angle;         // innovation (residual)
    angle += K0 * y;
    bias  += K1 * y;

    float P00_tmp = P[0][0];
    float P01_tmp = P[0][1];
    P[0][0] -= K0 * P00_tmp;
    P[0][1] -= K0 * P01_tmp;
    P[1][0] -= K1 * P00_tmp;
    P[1][1] -= K1 * P01_tmp;

    return angle;
  }
};

// ── Globals ───────────────────────────────────────────────────────────────────
Kalman kalmanY;
uint32_t lastTime;

// ── Helpers ───────────────────────────────────────────────────────────────────
void mpuWrite(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

// Reads all 7 × 16-bit raw values in one burst (14 bytes).
void mpuRead(int16_t &ax, int16_t &ay, int16_t &az,
             int16_t &gx, int16_t &gy, int16_t &gz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(REG_ACCEL_XOUT);
  Wire.endTransmission(false);          // repeated-start
  Wire.requestFrom(MPU_ADDR, (uint8_t)14);

  ax = Wire.read() << 8 | Wire.read();
  ay = Wire.read() << 8 | Wire.read();
  az = Wire.read() << 8 | Wire.read();
  Wire.read(); Wire.read();             // discard temperature
  gx = Wire.read() << 8 | Wire.read();
  gy = Wire.read() << 8 | Wire.read();
  gz = Wire.read() << 8 | Wire.read();
}

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(9600);
  Wire.begin();

  // Wake the MPU-6050 (clear SLEEP bit)
  mpuWrite(REG_PWR_MGMT, 0x00);
  delay(100);

  // ── Compute the initial accelerometer angle to seed the filter ────────────
  int16_t ax, ay, az, gx, gy, gz;
  mpuRead(ax, ay, az, gx, gy, gz);

  float accX = ax / ACCEL_SCALE;
  float accZ = az / ACCEL_SCALE;
  // Pitch = atan2(ax, -az)  — pins-up mounting (Z and X inverted)
  float initAngle = atan2(accX, -accZ) * RAD_TO_DEG;
  kalmanY.angle = initAngle;

  lastTime = micros();

  Serial.println(F("time_ms,accel_angle,gyro_rate,kalman_angle"));
}

// ── Loop ──────────────────────────────────────────────────────────────────────
void loop() {
  int16_t ax, ay, az, gx, gy, gz;
  mpuRead(ax, ay, az, gx, gy, gz);

  uint32_t now = micros();
  float dt = (now - lastTime) * 1e-6f;  // seconds
  lastTime = now;

  // Convert raw values
  float accX  =  ax / ACCEL_SCALE;   // [g]
  float accZ  =  az / ACCEL_SCALE;   // [g]
  float gyroY =  gy / GYRO_SCALE;    // [°/s]  — Y-axis gyro = pitch rate

  // Accelerometer angle — pins-up: Z and X are inverted
  float accelAngle = atan2(accX, -accZ) * RAD_TO_DEG;

  // Gyro Y rate — invert sign because board is flipped
  float gyroYcorrected = -gyroY;

  // Kalman filter update
  float kalmanAngle = kalmanY.update(accelAngle, gyroYcorrected, dt);

  // ── Serial output (CSV — easy to plot in Serial Plotter) ──────────────────
  Serial.print(millis());
  Serial.print(',');
  Serial.print(accelAngle, 2);
  Serial.print(',');
  Serial.print(gyroY, 2);
  Serial.print(',');
  Serial.println(kalmanAngle, 2);

  delay(1000);   // ~100 Hz
}
