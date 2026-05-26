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

