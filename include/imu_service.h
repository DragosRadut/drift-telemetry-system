#pragma once

#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

extern Adafruit_MPU6050 mpu;

void readIMU();
bool calibrateIMU(int samples = 300, int delayMs = 10);
bool isMotionStillEnoughForCalibration();
