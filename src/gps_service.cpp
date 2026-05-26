#include "gps_service.h"

#include <math.h>
#include <HardwareSerial.h>

#include "config.h"
#include "models.h"

TinyGPSPlus gps;
static HardwareSerial gpsSerial(2);  // UART2: RX=GPS_RX_PIN, TX=GPS_TX_PIN

// =====================
// Internal helpers
// =====================

// Wrap angle to [0, 360)
static float wrapAngle(float a) {
    while (a >= 360.0f) a -= 360.0f;
    while (a <    0.0f) a += 360.0f;
    return a;
}

// Wrap angle difference to [-180, 180]
static float wrapTo180(float a) {
    while (a >  180.0f) a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
}

// =====================
// GPS read
// =====================

void initGPS() {
    gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    gpsOk = true;
    Serial.println("GPS: UART2 started (RX=" + String(GPS_RX_PIN) + " TX=" + String(GPS_TX_PIN) + ")");
}

void readGPS() {
    // Feed all available bytes to TinyGPS++ — non-blocking, called every loop tick.
    while (gpsSerial.available()) {
        gps.encode(gpsSerial.read());
    }

    // A fix is considered valid only if location, speed, and course are all
    // present and the last update was recent enough to be trusted.
    gpsValid = gps.location.isValid()
            && gps.speed.isValid()
            && gps.course.isValid()
            && gps.location.age() < 2000;

    if (gps.speed.isValid())    gpsSpeedKmh = gps.speed.kmph();
    if (gps.course.isValid())   gpsCourse   = gps.course.deg();
    if (gps.location.isValid()) {
        gpsLat = gps.location.lat();
        gpsLon = gps.location.lng();
    }
}

// =====================
// Heading fusion + drift angle
// Called every IMU tick (~100 ms) right after readIMU().
// =====================

void updateHeading() {
    static unsigned long lastMs       = 0;
    static bool          initialized  = false;

    unsigned long now = millis();

    // First call: just record the time and return.
    if (lastMs == 0) {
        lastMs = now;
        return;
    }

    float dt = (now - lastMs) / 1000.0f;
    lastMs = now;

    // Clamp dt: ignore unreasonably large gaps (device was halted/calibrating).
    if (dt <= 0.0f || dt > 0.5f) return;

    // ── Step 1: Initialize heading from GPS the first time we have reliable motion.
    //    Until then, integrate gyro from 0 (relative, not absolute).
    if (!initialized) {
        if (gpsValid && gpsSpeedKmh >= GPS_MIN_SPEED_KMH) {
            vehicleHeading = gpsCourse;
            initialized    = true;
            Serial.printf("GPS heading init: %.1f deg\n", vehicleHeading);
        }
        // Still integrate gyro so heading isn't frozen at 0 on the dashboard.
        vehicleHeading = wrapAngle(vehicleHeading + yawRateDps * dt);
        return;
    }

    // ── Step 2: Integrate gyro Z for fast yaw changes.
    //    ψ(k) = ψ(k-1) + r_gyro * dt
    vehicleHeading = wrapAngle(vehicleHeading + yawRateDps * dt);

    // ── Step 3: GPS course correction — complementary filter.
    //    ψ_filtered = α * ψ_integrated + (1-α) * χ_gps
    //    Implemented as: ψ += (1-α) * angularDiff(χ_gps, ψ)
    //    to handle the wrap-around at 0/360 correctly.
    if (gpsValid && gpsSpeedKmh >= GPS_MIN_SPEED_KMH) {
        float diff = wrapTo180(gpsCourse - vehicleHeading);
        vehicleHeading = wrapAngle(vehicleHeading + (1.0f - GPS_HEADING_ALPHA) * diff);
    }

    // ── Step 4: Drift angle β = wrapTo180(χ_gps - ψ_vehicle), with low-pass filter.
    if (gpsValid && gpsSpeedKmh >= GPS_MIN_SPEED_KMH) {
        float betaRaw = wrapTo180(gpsCourse - vehicleHeading);
        driftAngle   += DRIFT_ANGLE_LP_ALPHA * (betaRaw - driftAngle);
    } else {
        // Below speed threshold or no GPS: decay smoothly toward zero.
        driftAngle *= 0.92f;
        if (fabsf(driftAngle) < 0.1f) driftAngle = 0.0f;
    }
}
