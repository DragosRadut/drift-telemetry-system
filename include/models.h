#pragma once

#include <Arduino.h>

// =====================
// State model
// =====================
enum DeviceState {
  STATE_BOOT,
  STATE_CALIBRATING,
  STATE_READY,
  STATE_RUNNING,
  STATE_ERROR
};

const char* stateToString(DeviceState s);

// =====================
// IMU thresholds defaults
// overwritten from /config when available
// =====================
extern float cfgRunStartSpeedKmh;
extern uint32_t cfgRunEndDelayMs;
extern float cfgYawRateThreshold;
extern float cfgLatGThreshold;
extern float cfgDriftAngleThreshold;

// =====================
// Globals (shared state)
// =====================
extern bool imuOk;
extern bool oledOk;
extern bool wifiOk;
extern bool firebaseReady;

extern DeviceState currentState;

extern float gyroBiasX;
extern float gyroBiasY;
extern float gyroBiasZ;

extern float accBiasX;
extern float accBiasY;
extern float accBiasZ;

extern float yawRateDps;
extern float latG;
extern float temperatureC;

// GPS data (updated by gps_service)
extern bool  gpsOk;
extern bool  gpsValid;        // fix present and < 2 s old
extern float gpsSpeedKmh;
extern float gpsCourse;       // course-over-ground χ [0..360)
extern float gpsLat;
extern float gpsLon;

// Derived telemetry (updated by updateHeading each IMU tick)
extern float vehicleHeading;  // fused heading ψ [0..360)
extern float driftAngle;      // sideslip β, degrees

extern bool calibrated;
extern bool runActive;
extern bool drifting;

extern String warningText;
extern String errorText;

extern String currentRunId;
extern unsigned long currentRunStartedAt;
extern unsigned long recordStartMs;
extern unsigned long lastDriftLikeMs;
extern unsigned long driftCandidateStartedMs;

extern uint32_t lastAckRequestId;
extern uint32_t lastSeenCommandRequestId;

extern unsigned long lastSensorMs;
extern unsigned long lastStatusMs;
extern unsigned long lastLiveUploadMs;
extern unsigned long lastCommandPollMs;
extern unsigned long lastConfigPollMs;
extern unsigned long lastOledMs;
extern unsigned long lastRunSampleUploadMs;
