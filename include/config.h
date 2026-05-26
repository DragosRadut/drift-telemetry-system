#pragma once

#include <Arduino.h>

// =====================
// User configuration
// =====================
#define WIFI_SSID            "K."
#define WIFI_PASSWORD        "09102002"

#define API_KEY              "AIzaSyChgkRxKS5-YitGEnh-L-L4IDuHdOVftPE"
#define DATABASE_URL         "https://drift-telemetry-79445-default-rtdb.europe-west1.firebasedatabase.app"
#define USER_EMAIL           "admin@wolfmotorsport.ro"
#define USER_PASSWORD        "dragos2002"

#define DEVICE_ID            "drift-01"
#define FIRMWARE_VERSION     "0.3.0-imuproto"

// =====================
// Pins / hardware
// =====================
#define SDA_PIN 21
#define SCL_PIN 22

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_ADDR 0x3C

#define GPS_RX_PIN  16
#define GPS_TX_PIN  17
#define GPS_BAUD    9600

// =====================
// Timing
// =====================
static const uint32_t SENSOR_PERIOD_MS      = 100;   // 10 Hz local sensing
static const uint32_t STATUS_PERIOD_MS      = 2000;  // 0.5 Hz status heartbeat
static const uint32_t LIVE_UPLOAD_MS        = 750;   // ~1.3 Hz cloud live telemetry
static const uint32_t COMMAND_POLL_MS       = 1000;  // 1 Hz remote command checks
static const uint32_t CONFIG_POLL_MS        = 5000;  // config changes are infrequent
static const uint32_t OLED_REFRESH_MS       = 250;   // smooth local display
static const uint32_t RUN_SAMPLE_UPLOAD_MS  = 1000;  // 1 Hz run logging
static const uint32_t STATIONARY_CHECK_MS   = 4000;  // more stable recalibration decision

// GPS / heading fusion
static const float GPS_MIN_SPEED_KMH         = 5.0f;   // below this GPS course is unreliable
static const float GPS_HEADING_ALPHA         = 0.95f;  // complementary filter: weight on gyro
static const float DRIFT_ANGLE_LP_ALPHA      = 0.15f;  // low-pass smoothing for β

// IMU-only provisional thresholds
static const float SPEED_PLACEHOLDER_KMH     = 25.0f;  // removed once GPS is live
static const uint32_t RUN_START_HOLD_MS      = 700;
static const float CALIBRATION_YAW_BIAS_DPS  = 1.5f;
static const float STATIONARY_GYRO_DPS       = 1.2f;
static const float STATIONARY_LATG_G         = 0.08f;
