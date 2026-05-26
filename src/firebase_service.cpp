#include "firebase_service.h"

#include <math.h>
#include <ArduinoJson.h>

#include "config.h"
#include "models.h"
#include "path_utils.h"
#include "display_service.h"
#include "imu_service.h"

// =====================
// Firebase globals
// =====================
// Auth/client objects vary slightly by FirebaseClient version.
// This layout follows the maintained library style conceptually.
UserAuth user_auth(API_KEY, USER_EMAIL, USER_PASSWORD);
FirebaseApp app;
WiFiClientSecure ssl;
AsyncClientClass aClient(ssl);
RealtimeDatabase Database;

void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);                 // phone hotspots drop idle clients
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 20000) {
    delay(250);
    if (oledOk) {
      drawStateScreen("BOOT", "Connecting WiFi...", WiFi.SSID(), String((millis() - start) / 1000) + "s");
    }
  }

  wifiOk = WiFi.status() == WL_CONNECTED;
  if (!wifiOk) {
    warningText = "WiFi disconnected";
    return;
  }

  // Phone hotspots often hand out broken DNS (empty, gateway-only, or carrier-NAT).
  // Force public DNS unconditionally so identitytoolkit.googleapis.com resolves.
  WiFi.config(WiFi.localIP(), WiFi.gatewayIP(), WiFi.subnetMask(),
              IPAddress(8, 8, 8, 8), IPAddress(1, 1, 1, 1));

  Serial.printf("WiFi OK  IP=%s  GW=%s  DNS=%s  RSSI=%d\n",
                WiFi.localIP().toString().c_str(),
                WiFi.gatewayIP().toString().c_str(),
                WiFi.dnsIP(0).toString().c_str(),
                WiFi.RSSI());

  // Sanity-check DNS *before* we let Firebase fight with it.
  IPAddress probe;
  if (WiFi.hostByName("identitytoolkit.googleapis.com", probe)) {
    Serial.printf("DNS probe OK: identitytoolkit -> %s\n", probe.toString().c_str());
  } else {
    Serial.println("DNS probe FAILED for identitytoolkit.googleapis.com");
    warningText = "DNS broken on this network";
  }
}

void initFirebase() {
  ssl.setInsecure();

  Serial.println("Firebase: initializeApp...");
  initializeApp(aClient, app, getAuth(user_auth));
  app.getApp<RealtimeDatabase>(Database);
  Database.url(DATABASE_URL);

  // Auth can need >15s on a slow phone hotspot; give it 45s and log progress.
  unsigned long start = millis();
  unsigned long lastTick = 0;
  while (!app.ready() && millis() - start < 45000) {
    app.loop();
    if (millis() - lastTick > 2000) {
      lastTick = millis();
      Serial.printf("Firebase: waiting auth... %lus  lastErr=\"%s\" code=%d\n",
                    (millis() - start) / 1000,
                    aClient.lastError().message().c_str(),
                    aClient.lastError().code());
    }
    delay(10);
  }

  firebaseReady = app.ready();
  if (firebaseReady) {
    Serial.println("Firebase: AUTH OK, app.ready() = true");
  } else {
    Serial.printf("Firebase: AUTH FAILED. lastErr=\"%s\" code=%d\n",
                  aClient.lastError().message().c_str(),
                  aClient.lastError().code());
    warningText = "Firebase auth not ready";
  }
}

void publishMetaOnce() {
  if (!firebaseReady) return;

  Database.set<String>(aClient, metaPath() + "/name", "Drift Telemetry Prototype");
  Database.set<String>(aClient, metaPath() + "/firmware", FIRMWARE_VERSION);
}

void publishStatus() {
  if (!firebaseReady) return;

  // ArduinoJson builds the full object in one shot; object_t(payload) hands
  // the raw JSON string straight to the RTDB — single write, no extra traffic.
  StaticJsonDocument<512> doc;
  doc["state"]               = stateToString(currentState);
  doc["online"]              = wifiOk;
  doc["calibrated"]          = calibrated;
  doc["lastSeen"]            = (long long)time(nullptr) * 1000LL;
  doc["wifiRssi"]            = (int)WiFi.RSSI();
  doc["currentRunId"]        = currentRunId.length() ? currentRunId.c_str() : "-";
  doc["currentRunStartedAt"] = currentRunStartedAt ? (long long)currentRunStartedAt * 1000LL : 0;
  doc["lastAckRequestId"]    = (int)lastAckRequestId;
  doc["warning"]             = warningText.length() ? warningText.c_str() : "-";
  doc["error"]               = errorText.length()   ? errorText.c_str()   : "-";
  String payload;
  serializeJson(doc, payload);

  bool ok = Database.set<object_t>(aClient, statusPath(), object_t(payload));
  if (!ok) {
    Serial.printf("publishStatus FAILED: %s (code %d)\n",
                  aClient.lastError().message().c_str(),
                  aClient.lastError().code());
  }
}

void publishLive() {
  if (!firebaseReady) return;

  StaticJsonDocument<512> doc;
  doc["timestamp"]   = (long long)time(nullptr) * 1000LL;
  doc["speedKmh"]    = gpsSpeedKmh;
  doc["yawRate"]     = yawRateDps;
  doc["latG"]        = latG;
  doc["driftAngle"]  = driftAngle;
  doc["drifting"]        = drifting;
  doc["runActive"]       = runActive;
  doc["temperature"]     = temperatureC;
  doc["gpsValid"]        = gpsValid;
  doc["lat"]             = gpsLat;
  doc["lon"]             = gpsLon;
  doc["vehicleHeading"]  = vehicleHeading;
  String payload;
  serializeJson(doc, payload);

  bool ok = Database.set<object_t>(aClient, livePath(), object_t(payload));
  if (!ok) {
    Serial.printf("publishLive FAILED: %s (code %d)\n",
                  aClient.lastError().message().c_str(),
                  aClient.lastError().code());
  }
}

static float    _runMaxYawRate  = 0.0f;
static float    _runMaxLatG     = 0.0f;
static float    _runTopSpeed    = 0.0f;
static float    _runSpeedSum    = 0.0f;
static float    _runTopAngle    = 0.0f;
static float    _runAngleSum    = 0.0f;
static uint32_t _runSampleCount = 0;

void resetRunAccumulators() {
  _runMaxYawRate  = 0.0f;
  _runMaxLatG     = 0.0f;
  _runTopSpeed    = 0.0f;
  _runSpeedSum    = 0.0f;
  _runTopAngle    = 0.0f;
  _runAngleSum    = 0.0f;
  _runSampleCount = 0;
}

void publishRunSample() {
  if (!firebaseReady || !runActive || currentRunId.isEmpty()) return;

  _runSampleCount++;
  _runMaxYawRate = max(_runMaxYawRate, fabs(yawRateDps));
  _runMaxLatG    = max(_runMaxLatG,    fabs(latG));
  _runTopSpeed   = max(_runTopSpeed,   gpsSpeedKmh);
  _runSpeedSum  += gpsSpeedKmh;
  float absAngle = fabs(driftAngle);
  _runTopAngle   = max(_runTopAngle, absAngle);
  _runAngleSum  += absAngle;

  String sampleKey = String(millis());

  StaticJsonDocument<512> sampleDoc;
  sampleDoc["timestamp"]      = (long long)time(nullptr) * 1000LL;
  sampleDoc["speedKmh"]       = gpsSpeedKmh;
  sampleDoc["yawRate"]        = yawRateDps;
  sampleDoc["latG"]           = latG;
  sampleDoc["driftAngle"]     = driftAngle;
  sampleDoc["drifting"]       = drifting;
  sampleDoc["lat"]            = gpsLat;
  sampleDoc["lon"]            = gpsLon;
  sampleDoc["heading"]        = vehicleHeading;
  sampleDoc["gpsValid"]       = gpsValid;
  String samplePayload;
  serializeJson(sampleDoc, samplePayload);
  Database.set<object_t>(aClient,
    runsPath() + "/" + currentRunId + "/samples/" + sampleKey,
    object_t(samplePayload));

  StaticJsonDocument<256> summaryDoc;
  summaryDoc["sampleCount"] = _runSampleCount;
  summaryDoc["maxYawRate"]  = _runMaxYawRate;
  summaryDoc["maxLatG"]     = _runMaxLatG;
  summaryDoc["topSpeed"]    = _runTopSpeed;
  summaryDoc["avgSpeed"]    = _runSpeedSum / _runSampleCount;
  summaryDoc["topAngle"]    = _runTopAngle;
  summaryDoc["avgAngle"]    = _runAngleSum / _runSampleCount;
  String summaryPayload;
  serializeJson(summaryDoc, summaryPayload);
  Database.set<object_t>(aClient,
    runsPath() + "/" + currentRunId + "/summary",
    object_t(summaryPayload));
}

void applyCommand(const String& action, uint32_t requestId) {
  if (requestId <= lastSeenCommandRequestId) return;

  lastSeenCommandRequestId = requestId;
  
  if (action == "calibrate") {
    calibrateIMU();
    lastAckRequestId = requestId;
  } else if (action == "reset") {
    lastAckRequestId = requestId;
    ESP.restart();
  } else {
    lastAckRequestId = requestId;
  }
}

void pollCommands() {
  if (!firebaseReady) return;

  // Read the whole /commands node as JSON/text and parse fields locally.
  String payload = Database.get<String>(aClient, commandsPath());

  if (payload.length() == 0 || payload == "null") return;

  DynamicJsonDocument doc(256);
  DeserializationError err = deserializeJson(doc, payload);
  if (err) {
    Serial.printf("commands JSON parse error: %s\n", err.c_str());
    return;
  }

  String action = doc["action"] | "";
  uint32_t requestId = doc["requestId"] | 0;

  if (requestId > 0 && action.length()) {
    applyCommand(action, requestId);
  }
}

void pollConfig() {
  if (!firebaseReady) return;

  String payload = Database.get<String>(aClient, configPath());

  if (payload.length() == 0 || payload == "null") return;

  DynamicJsonDocument doc(512);
  DeserializationError err = deserializeJson(doc, payload);
  if (err) {
    Serial.printf("config JSON parse error: %s\n", err.c_str());
    return;
  }

  float runStartSpeed = doc["runStartSpeedKmh"] | cfgRunStartSpeedKmh;
  uint32_t runEndDelay = doc["runEndDelayMs"] | cfgRunEndDelayMs;
  float yawThr = doc["yawRateThreshold"] | cfgYawRateThreshold;
  float latGThr = doc["latGThreshold"] | cfgLatGThreshold;
  float driftAngleThr = doc["driftAngleThreshold"] | cfgDriftAngleThreshold;

  if (runStartSpeed > 0) cfgRunStartSpeedKmh = runStartSpeed;
  if (runEndDelay > 0) cfgRunEndDelayMs = runEndDelay;
  if (yawThr > 0) cfgYawRateThreshold = yawThr;
  if (latGThr > 0) cfgLatGThreshold = latGThr;
  if (driftAngleThr > 0) cfgDriftAngleThreshold = driftAngleThr;
}
