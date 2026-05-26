#pragma once

#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

extern Adafruit_SSD1306 display;

void drawStateScreen(const String& title,
                     const String& line1 = "",
                     const String& line2 = "",
                     const String& line3 = "");

void refreshOLED();
