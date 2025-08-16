#include <Wire.h>
#include <MPU6050_light.h>      // rfetick library
#include <MadgwickAHRS.h>       // Arduino Madgwick
#include <BleMouse.h>

// --- BLE Mouse ---
BleMouse bleMouse("AirMouse ESP32", "ESP32", 100);

// --- IMU ---
MPU6050 mpu(Wire);
Madgwick filter;

// --- Pins ---
#define LEFTBUTTON   19
#define RIGHTBUTTON  18

// --- Sampling settings ---
const float SAMPLE_FREQ_HZ = 100.0;
const uint32_t SAMPLE_PERIOD_US = 1000000.0 / SAMPLE_FREQ_HZ;

// --- State ---
uint32_t lastUpdate = 0;
float alpha = 0.2; // smoothing factor
float roll_s = 0, pitch_s = 0;

// --- Sensitivity ---
float sensitivity = 3.0;  // pixels per degree per frame

// Button helpers
inline bool pressed(int pin) { return digitalRead(pin) == LOW; }
inline int8_t clampMove(int v) {
  if (v > 127) return 127;
  if (v < -127) return -127;
  return (int8_t)v;
}

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);

  pinMode(LEFTBUTTON, INPUT_PULLUP);
  pinMode(RIGHTBUTTON, INPUT_PULLUP);

  byte status = mpu.begin();
  Serial.print("MPU6050 status: "); Serial.println(status);
  while (status != 0) { delay(100); }

  Serial.println("Calibrating...");
  delay(1000);
  mpu.calcOffsets();   // calibrate accel + gyro
  Serial.println("Calibration done.");

  filter.begin(SAMPLE_FREQ_HZ);

  bleMouse.begin();

  lastUpdate = micros();
}

void loop() {
  uint32_t now = micros();
  if ((now - lastUpdate) < SAMPLE_PERIOD_US) return;
  lastUpdate += SAMPLE_PERIOD_US;

  // --- Update IMU ---
  mpu.update();

  float ax = mpu.getAccX();
  float ay = mpu.getAccY();
  float az = mpu.getAccZ();
  float gx_dps = mpu.getGyroX();
  float gy_dps = mpu.getGyroY();
  float gz_dps = mpu.getGyroZ();

  const float DEG2RAD = 0.017453292519943295f;
  filter.updateIMU(gx_dps * DEG2RAD, gy_dps * DEG2RAD, gz_dps * DEG2RAD,
                   ax, ay, az);

  float roll  = filter.getRoll();
  float pitch = filter.getPitch();

  // Smoothing
  roll_s  = alpha * roll  + (1 - alpha) * roll_s;
  pitch_s = alpha * pitch + (1 - alpha) * pitch_s;

  // --- Mouse Control ---
  if (bleMouse.isConnected()) {
    // Map roll & pitch deltas to cursor
    static float lastRoll = roll_s;
    static float lastPitch = pitch_s;

    float dRoll = roll_s - lastRoll;
    float dPitch = pitch_s - lastPitch;
    lastRoll = roll_s;
    lastPitch = pitch_s;

    int dx = (int)(dRoll * sensitivity);
    int dy = (int)(-dPitch * sensitivity);  // invert for natural feel

    if (dx || dy) {
      bleMouse.move(clampMove(dx), clampMove(dy), 0);
    }

    // Buttons
    static bool lastLeft=false, lastRight=false;
    bool leftNow = pressed(LEFTBUTTON);
    bool rightNow = pressed(RIGHTBUTTON);

    if (leftNow && !lastLeft) bleMouse.press(MOUSE_LEFT);
    if (!leftNow && lastLeft) bleMouse.release(MOUSE_LEFT);

    if (rightNow && !lastRight) bleMouse.press(MOUSE_RIGHT);
    if (!rightNow && lastRight) bleMouse.release(MOUSE_RIGHT);

    lastLeft = leftNow;
    lastRight = rightNow;
  }
}
