#include "imu_service.h"

#include <Wire.h>
#include <math.h>

#include "config.h"
#include "models.h"
#include "display_service.h"
#include "firebase_service.h"

Adafruit_MPU6050 mpu;

bool isMotionStillEnoughForCalibration() {
  return fabs(yawRateDps) < STATIONARY_GYRO_DPS && fabs(latG) < STATIONARY_LATG_G;
}

void readIMU() {
  sensors_event_t a, g, t;
  mpu.getEvent(&a, &g, &t);

  yawRateDps = (g.gyro.z - gyroBiasZ) * 57.2958f;
  latG = (a.acceleration.y - accBiasY) / 9.81f;
  temperatureC = t.temperature;
}

bool calibrateIMU(int samples, int delayMs) {
  if (!imuOk) return false;

  currentState = STATE_CALIBRATING;
  warningText = "";
  errorText = "";
  drawStateScreen("CALIBRATING", "Keep car still", "Settling sensor...");

  const uint32_t settleMs = 2000;
  unsigned long settleStart = millis();
  while (millis() - settleStart < settleMs) {
    app.loop();
    delay(10);
  }

  float sumAx = 0, sumAy = 0, sumAz = 0;
  float sumGx = 0, sumGy = 0, sumGz = 0;
  int collected = 0;

  for (int i = 0; i < samples; i++) {
    sensors_event_t a, g, t;
    mpu.getEvent(&a, &g, &t);

    sumAx += a.acceleration.x;
    sumAy += a.acceleration.y;
    sumAz += a.acceleration.z;

    sumGx += g.gyro.x;
    sumGy += g.gyro.y;
    sumGz += g.gyro.z;

    collected++;

    if (oledOk && i % 25 == 0) {
      display.clearDisplay();
      display.setCursor(0, 0);
      display.println("CALIBRATING");
      display.println("----------------");
      display.println("Keep car still");
      display.printf("Progress: %d%%\n", (i * 100) / samples);
      display.display();
    }

    app.loop();
    delay(delayMs);
  }

  if (collected == 0) {
    calibrated = false;
    currentState = STATE_ERROR;
    errorText = "No calibration samples";
    // Note: do NOT publishStatus() here. setup() hasn't returned yet, so
    // loop() isn't pumping app.loop() between sub-requests; a blocking RTDB
    // write here can stall long enough to trip the watchdog and silently reboot.
    return false;
  }


  accBiasX = sumAx / collected;
  accBiasY = sumAy / collected;
  accBiasZ = sumAz / collected;

  gyroBiasX = sumGx / collected;
  gyroBiasY = sumGy / collected;
  gyroBiasZ = sumGz / collected;

  calibrated = true;
  currentState = STATE_READY;
  errorText = "";

  float yawBiasDps = gyroBiasZ * 57.2958f;

  if (fabs(yawBiasDps) > CALIBRATION_YAW_BIAS_DPS) {
    warningText = "Yaw bias high; recalibrate when parked";
  } else {
    warningText = "";
  }

  Serial.println("Calibration done.");
  Serial.printf("Samples: %d\n", collected);
  Serial.printf("Acc bias: %.3f %.3f %.3f\n", accBiasX, accBiasY, accBiasZ);
  Serial.printf("Gyro bias: %.3f %.3f %.3f\n", gyroBiasX, gyroBiasY, gyroBiasZ);
  Serial.printf("Yaw bias dps: %.3f\n", yawBiasDps);

  publishStatus();
  return true;
}