#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <time.h>

#include "config.h"
#include "models.h"
#include "path_utils.h"
#include "imu_service.h"
#include "gps_service.h"
#include "display_service.h"
#include "state_machine.h"
#include "firebase_service.h"

// =====================
// IMU thresholds defaults
// overwritten from /config when available
// =====================
float cfgRunStartSpeedKmh      = 20.0f;
uint32_t cfgRunEndDelayMs      = 5000;
float cfgYawRateThreshold      = 8.0f;
float cfgLatGThreshold         = 0.20f;
float cfgDriftAngleThreshold   = 10.0f;   // not used yet in IMU-only phase

// =====================
// Globals
// =====================
bool imuOk = false;
bool oledOk = false;
bool wifiOk = false;
bool firebaseReady = false;

DeviceState currentState = STATE_BOOT;

float gyroBiasX = 0.0f;
float gyroBiasY = 0.0f;
float gyroBiasZ = 0.0f;

float accBiasX = 0.0f;
float accBiasY = 0.0f;
float accBiasZ = 0.0f;

float yawRateDps = 0.0f;
float latG = 0.0f;
float temperatureC = 0.0f;

bool  gpsOk       = false;
bool  gpsValid    = false;
float gpsSpeedKmh = 0.0f;
float gpsCourse   = 0.0f;
float gpsLat      = 0.0f;
float gpsLon      = 0.0f;

float vehicleHeading = 0.0f;
float driftAngle     = 0.0f;

bool calibrated = false;
bool runActive = false;
bool drifting = false;

String warningText = "";
String errorText = "";

String currentRunId = "";
unsigned long currentRunStartedAt = 0;
unsigned long recordStartMs = 0;
unsigned long lastDriftLikeMs = 0;
unsigned long driftCandidateStartedMs = 0;

uint32_t lastAckRequestId = 0;
uint32_t lastSeenCommandRequestId = 0;

unsigned long lastSensorMs = 0;
unsigned long lastStatusMs = 0;
unsigned long lastLiveUploadMs = 0;
unsigned long lastCommandPollMs = 0;
unsigned long lastConfigPollMs = 0;
unsigned long lastOledMs = 0;
unsigned long lastRunSampleUploadMs = 0;

// =====================
// Setup
// =====================
void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin(SDA_PIN, SCL_PIN);

  if (display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    oledOk = true;
    drawStateScreen("BOOT", "OLED OK", "Init IMU...");
  } else {
    Serial.println("SSD1306 not found");
  }

  if (!mpu.begin(0x68)) {
    Serial.println("Failed to find MPU6050!");
    currentState = STATE_ERROR;
    errorText = "MPU6050 not found";
    drawStateScreen("ERROR", "MPU6050 not found", "Check wiring");
    return;
  }

  imuOk = true;
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  connectWiFi();

  // Sync time over NTP before any TLS work (cert validation needs a valid clock).
  if (wifiOk) {
    configTime(0, 0, "pool.ntp.org", "time.nist.gov");
    time_t nowSec = 0;
    unsigned long t0 = millis();
    while (nowSec < 1700000000 && millis() - t0 < 8000) {
      delay(200);
      time(&nowSec);
    }
    Serial.printf("NTP epoch: %ld\n", (long)nowSec);
  }

  initGPS();

  initFirebase();

  publishMetaOnce();

  drawStateScreen("BOOT", "IMU OK", "Needs calibration");

  calibrateIMU();
}

// =====================
// Loop
// =====================
void loop() {
  wifiOk = WiFi.status() == WL_CONNECTED;

  // Only pump Firebase when Wi-Fi is up; otherwise it spams DNS/SSL errors.
  if (wifiOk) {
    app.loop();
  }

  if (!wifiOk && currentState != STATE_ERROR) {
    warningText = "WiFi disconnected";
  }

  unsigned long now = millis();

  // GPS bytes must be fed every loop tick to avoid UART buffer overflow.
  if (gpsOk) readGPS();

  if (imuOk && now - lastSensorMs >= SENSOR_PERIOD_MS) {
    lastSensorMs = now;
    readIMU();
    updateHeading();
    evaluateCalibrationNeed();
    updateRunDetection();

    Serial.printf("%s yaw=%.1f hdg=%.1f b=%.1f spd=%.1f v=%d | gps chars=%lu fix=%lu bad=%lu\n",
                  stateToString(currentState),
                  yawRateDps, vehicleHeading, driftAngle, gpsSpeedKmh, gpsValid,
                  gps.charsProcessed(), gps.sentencesWithFix(), gps.failedChecksum());
  }

  if (firebaseReady && now - lastStatusMs >= STATUS_PERIOD_MS) {
    lastStatusMs = now;
    publishStatus();
  }

  if (firebaseReady && now - lastLiveUploadMs >= LIVE_UPLOAD_MS) {
    lastLiveUploadMs = now;
    publishLive();
  }

  if (firebaseReady && now - lastRunSampleUploadMs >= RUN_SAMPLE_UPLOAD_MS) {
    lastRunSampleUploadMs = now;
    publishRunSample();
  }

  if (firebaseReady && now - lastCommandPollMs >= COMMAND_POLL_MS) {
    lastCommandPollMs = now;
    pollCommands();
  }

  if (firebaseReady && now - lastConfigPollMs >= CONFIG_POLL_MS) {
    lastConfigPollMs = now;
    pollConfig();
  }

  if (oledOk && now - lastOledMs >= OLED_REFRESH_MS) {
    lastOledMs = now;
    refreshOLED();
  }

  delay(5);
}
