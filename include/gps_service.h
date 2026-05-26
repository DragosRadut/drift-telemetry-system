#pragma once

#include <Arduino.h>
#include <TinyGPSPlus.h>

extern TinyGPSPlus gps;

void initGPS();
void readGPS();
void updateHeading();
