# ESP32 Drift Telemetry Firmware Scaffold

This is a modular firmware starter for the ESP32-based drift telemetry project. It currently targets an **IMU-only prototype** using:
- ESP32 DevKit
- MPU6050
- SSD1306 OLED
- Firebase Realtime Database over Wi-Fi

## Purpose

This version is meant to validate the full app-device loop before the GPS module arrives. It focuses on:
- IMU calibration
- Live yaw-rate and lateral-G telemetry
- Device state reporting
- Firebase command handling (`calibrate`, `reset`)
- Auto-created provisional runs based on IMU thresholds
- OLED local feedback

## Structure

- `platformio.ini` — PlatformIO board, framework and library config
- `include/config.h` — credentials, pins, timing, constants
- `include/models.h` — shared enums, state model and global declarations
- `include/path_utils.h` — Firebase path builders (`/devices/<id>/...`) and run-id helper
- `include/imu_service.h` — IMU init, read, calibration declarations
- `include/display_service.h` — OLED screens declarations
- `include/state_machine.h` — device state and run logic declarations
- `include/firebase_service.h` — RTDB read/write, Wi-Fi and Firebase init declarations
- `src/main.cpp` — top-level wiring and scheduler (defines globals, `setup()`, `loop()`)
- `src/imu_service.cpp` — IMU init, read, calibration
- `src/display_service.cpp` — OLED screens
- `src/state_machine.cpp` — device state, `stateToString()`, run start/finish/detection
- `src/firebase_service.cpp` — Wi-Fi connect, Firebase init, status/live/run publishers, command/config polling

## Notes

- Timestamps currently use `millis()` and should later be replaced with real time (NTP/GPS).
- `speedKmh` and `driftAngleDeg` are placeholders until GPS integration is added.
- Run detection is provisional and based only on yaw-rate and lateral-G thresholds.
- Firebase client APIs may need small adjustments depending on the installed library version.
``
## Next steps

No remaining placeholder uses. Now the lab testing roadmap:

Lab testing roadmap
Step 0 — Wire the NEO-6M

NEO-6M   →  ESP32
VCC      →  3.3 V  (most breakouts have their own regulator; use 5 V if yours doesn't)
GND      →  GND
TX       →  GPIO 16  (GPS_RX_PIN)
RX       →  GPIO 17  (GPS_TX_PIN — optional, you're not sending commands)
Step 1 — Verify raw NMEA (no code changes needed)
Before flashing the full firmware, open a serial terminal and check the serial monitor. With just initGPS() called and while (gpsSerial.available()) Serial.write(gpsSerial.read()); in the loop, you should see sentences like:


$GPRMC,123456.00,A,4455.12345,N,02634.56789,E,0.012,,,,,A*XX
$GPGGA,123456.00,4455.12345,N,...
If you see garbage or nothing, it's a wiring/baud issue. NEO-6M defaults to 9600 baud which matches GPS_BAUD.

Step 2 — Cold fix acquisition (near a window)
The NEO-6M needs sky view for first fix — typically 30–90 seconds outdoors, up to 5 minutes through glass. Watch the serial output for gpsValid=1 to appear. The OLED and serial line both now print v=0/v=1.

Lab shortcut: you don't need a full sky fix for most subsequent tests — you can inject fake values.

Step 3 — Heading initialization test (stationary, no GPS required)
Add this temporary override at the top of loop() for bench testing:


// LAB TEST: inject fake GPS motion to force heading init
gpsValid    = true;
gpsSpeedKmh = 25.0f;
gpsCourse   = 0.0f;
With those overrides in place, vehicleHeading should initialize to 0.0° and then track yawRateDps * dt. Rotate the IMU by hand and watch vehicleHeading change in the serial log:


READY yaw=45.2° hdg=45.2° β=0.0° spd=25.0 v=1, latG=0.001
Heading and yaw should agree while GPS course stays fixed at 0. driftAngle should converge toward 0 - heading.

Step 4 — Drift angle math test
With the fake GPS injected, set gpsCourse = 15.0f instead of 0.0f. Hold the IMU stationary — vehicleHeading should stay near 0° (no gyro rotation), and you should see driftAngle converge toward ~15° within a few seconds at DRIFT_ANGLE_LP_ALPHA = 0.15.

Then try gpsCourse = -20.0f (wraps to 340°) and verify driftAngle converges toward -20°. This validates the wrapTo180 logic.

Step 5 — Complementary filter check
With fake GPS at gpsCourse = 0°, rotate the IMU smoothly by about 30° and hold it. vehicleHeading should jump to ~30° from gyro integration. With GPS correction applying (α = 0.95, so 5% per tick toward GPS), heading should slowly pull back toward 0° over ~10 seconds. This verifies the filter is blending correctly and not snapping.

Step 6 — Firebase + dashboard end-to-end
Remove the fake GPS overrides and flash clean firmware. With a real GPS fix near a window:

Confirm gpsValid, lat, lon, speedKmh, driftAngle, vehicleHeading all appear in the Firebase live node
Confirm the dashboard map marker moves when you walk the device to a different room
driftAngle should be near 0° while walking straight, and show a small angle during turns
Step 7 — Run detection sanity check
Still indoors, manually exceed the thresholds by shaking the IMU:

|yawRateDps| > 8 AND |latG| > 0.2 for 700 ms → should trigger a run
Watch Firebase for a new runs/run-XXXX node to appear with startedAt in ms
The speed condition (cfgRunStartSpeedKmh = 20) won't be satisfied indoors with no GPS motion, so run detection won't auto-start — which is correct. You can temporarily lower cfgRunStartSpeedKmh to 0 via Firebase config to test the full run flow indoors.

What to check before going to the car
Check	Pass condition
gpsValid reaches true outdoors	Yes within 90 s
vehicleHeading initializes from GPS course	Serial prints "GPS heading init: X°"
Walking straight: driftAngle near 0°	< ±3° at walking speed
Turning corner: driftAngle spikes then returns	Visible spike, decays after turn
Run triggers on IMU shake (speed=0 mode)	Firebase runs/ node created
All live fields non-zero	Firebase live node has all 12 fields
