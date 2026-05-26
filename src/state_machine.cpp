#include "state_machine.h"

#include <math.h>
#include <time.h>
#include <ArduinoJson.h>

#include "config.h"
#include "models.h"
#include "imu_service.h"
#include "firebase_service.h"
#include "path_utils.h"

const char* stateToString(DeviceState s) {
  switch (s) {
    case STATE_BOOT: return "BOOT";
    case STATE_CALIBRATING: return "CALIBRATING";
    case STATE_READY: return "READY";
    case STATE_RUNNING: return "RUNNING";
    case STATE_ERROR: return "ERROR";
    default: return "ERROR";
  }
}

void evaluateCalibrationNeed() {
  if (currentState == STATE_CALIBRATING || currentState == STATE_RUNNING || currentState == STATE_ERROR) {
    return;
  }

  // Manual calibration only: do not force a special state.
  // If you want, keep a soft message only.
  if (calibrated && isMotionStillEnoughForCalibration()) {
    if (fabs(yawRateDps) > CALIBRATION_YAW_BIAS_DPS) {
      warningText = "Yaw drift high; recalibrate when parked";
    } else if (warningText == "Yaw drift high; recalibrate when parked") {
      warningText = "";
    }
  }

  if (currentState != STATE_RUNNING && currentState != STATE_CALIBRATING && currentState != STATE_ERROR) {
    currentState = STATE_READY;
  }
}

void startRun() {
  runActive = true;
  currentState = STATE_RUNNING;
  recordStartMs = millis();
  currentRunStartedAt = (unsigned long)time(nullptr);  // seconds; ×1000 applied on publish
  currentRunId = makeRunId();
  lastDriftLikeMs = millis();
  driftCandidateStartedMs = 0;
  resetRunAccumulators();

  Serial.printf("Run started: %s\n", currentRunId.c_str());

  if (firebaseReady) {
    StaticJsonDocument<512> doc;
    doc["startedAt"]  = (long long)time(nullptr) * 1000LL;
    doc["endedAt"]    = 0;
    doc["durationMs"] = 0;
    doc["startLat"]   = gpsLat;
    doc["startLon"]   = gpsLon;
    JsonObject summary = doc["summary"].to<JsonObject>();
    summary["topSpeed"]           = 0;
    summary["avgSpeed"]           = 0;
    summary["topAngle"]           = 0;
    summary["avgAngle"]           = 0;
    summary["maxYawRate"]         = fabs(yawRateDps);
    summary["maxLatG"]            = fabs(latG);
    summary["sampleCount"]        = 0;
    summary["hadNonDriftMoments"] = false;
    String payload;
    serializeJson(doc, payload);
    Database.set<object_t>(aClient, runsPath() + "/" + currentRunId, object_t(payload));
  }
}

void finishRun() {
  if (!runActive) return;

  runActive = false;
  drifting = false;
  currentState = STATE_READY;

  unsigned long durationMs = millis() - recordStartMs;

  Serial.printf("Run ended: %s (%lu ms)\n", currentRunId.c_str(), durationMs);

  if (firebaseReady && currentRunId.length()) {
    Database.set<object_t>(aClient, runsPath() + "/" + currentRunId + "/endedAt",
        object_t(String((long long)time(nullptr) * 1000LL)));
    Database.set<int>(aClient, runsPath() + "/" + currentRunId + "/durationMs", (int)durationMs);
  }

  currentRunId = "";
  currentRunStartedAt = 0;
  driftCandidateStartedMs = 0;
}

void updateRunDetection() {
  if (currentState == STATE_CALIBRATING || currentState == STATE_ERROR) {
    if (runActive) finishRun();
    return;
  }

  // drifting is a per-sample flag derived from IMU alone, independent of run state.
  drifting = fabs(yawRateDps) >= cfgYawRateThreshold &&
             fabs(latG)       >= cfgLatGThreshold;

  // Run lifecycle is speed-only: record every drive, let samples carry the drifting flag.
  bool moving = gpsSpeedKmh >= cfgRunStartSpeedKmh;

  if (!runActive) {
    if (moving) {
      if (driftCandidateStartedMs == 0) driftCandidateStartedMs = millis();
      if (millis() - driftCandidateStartedMs >= RUN_START_HOLD_MS) {
        startRun();
      }
    } else {
      driftCandidateStartedMs = 0;
    }
  } else {
    if (moving) {
      lastDriftLikeMs = millis();
    } else if (millis() - lastDriftLikeMs >= cfgRunEndDelayMs) {
      finishRun();
    }
  }
}